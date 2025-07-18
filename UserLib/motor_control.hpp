/*
 * motor_control.hpp
 *
 *  Created on: Jul 12, 2025
 *      Author: gomas
 */

#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

#include "CommonLib/Protocol/id_defines.hpp"
#include "CommonLib/Math/pid.hpp"
#include "CommonLib/Math/disturbance_observer.hpp"
#include "motor_model.hpp"
#include "C6x0_encoder.hpp"
#include "AMT21x_encoder.hpp"

#include <optional>

//TODO:MotorType == VESCの時どうするか考える
namespace BoardLib{

enum class RobomasMD{
	C610,
	C620
};

class RobomasMotorParam{
	//ロボマスモーターのモーターパラメータ
	//TODO:値の確認
	static constexpr float torque_coef_inv[2] = {1.0f/0.18f, 1.0f/0.3f}; //トルク定数の逆数[A/(N/m)]
	static constexpr float gear_ratio[2] = {36.0f,3591.0f/187.0f};
public:
	static constexpr float get_gear_ratio(RobomasMD m_type){
		return gear_ratio[static_cast<size_t>(m_type)];
	}
	static constexpr float torque_to_current(RobomasMD m_type, float torque){
		return torque * torque_coef_inv[static_cast<size_t>(m_type)];
	}
	static constexpr float current_to_torque(RobomasMD m_type, float current){
		return current / torque_coef_inv[static_cast<size_t>(m_type)];
	}
};


class C6x0Controller{
private:
	const size_t motor_id;
	RobomasMD motor_type;
	MReg::ControlMode mode;
	bool dob_enable;
	bool using_abs_enc;

	float torque = 0.0f;
	float target_rad = 0.0f;
	float target_speed = 0.0f;

public:
	CommonLib::Math::PIDController spd_pid;
	CommonLib::Math::PIDController pos_pid;
	CommonLib::Math::DisturbanceObserver<BoardLib::MotorInverceModel> dob;
	BoardLib::C6x0Enc enc;
	BoardLib::AMT21xEnc abs_enc;

	C6x0Controller(
			size_t _motor_id,
			RobomasMD _m_type,
			MReg::ControlMode _mode,
			bool _dob_en,
			bool _using_abs_enc,
			CommonLib::Math::PIDController&& _spd_pid,
			CommonLib::Math::PIDController&& _pos_pid,
			CommonLib::Math::DisturbanceObserver<BoardLib::MotorInverceModel>&& _dob,
			BoardLib::C6x0Enc&& _enc,
			BoardLib::AMT21xEnc&& _abs_enc)
	:motor_id(_motor_id),
	 motor_type(_m_type),
	 mode(_mode),
	 dob_enable(_dob_en),
	 using_abs_enc(_using_abs_enc),
	 spd_pid(std::move(_spd_pid)),
	 pos_pid(std::move(_pos_pid)),
	 dob(std::move(_dob)),
	 enc(std::move(_enc)),
	 abs_enc(std::move(_abs_enc)){
	}

	void set_motor_type(RobomasMD m_type){
		motor_type = m_type;
		enc.set_gear_ratio(RobomasMotorParam::get_gear_ratio(m_type));
	}
	RobomasMD get_motor_type(void) const {return motor_type;}

	void set_control_mode(MReg::ControlMode m);
	MReg::ControlMode get_control_mode(void)const{ return mode; }

	void  set_torque(float _torqeu){torque = _torqeu;}
	float get_torque(void) const {return torque;}

	void set_target_speed_rad(float _target_speed){ target_speed = _target_speed;}
	float get_target_speed_rad(void) const {return target_speed;}

	void set_target_rad(float _target_rad){ target_rad = _target_rad;}
	float get_target_rad(void) const {return target_rad;}

	//CANの受信割込みで呼び出し
	float pid_operation(const CommonLib::CanFrame &frame); //return torque
};


///////////////////////////////////////////////////////////////////////////////
//builder
///////////////////////////////////////////////////////////////////////////////

class C6x0ControllerBuilder{
private:
	size_t motor_id = 0;
	RobomasMD m_type = RobomasMD::C610;
	MReg::ControlMode c_mode = MReg::ControlMode::OPEN_LOOP;
	float update_freq = 1000.0f;

	float spd_kp = 0.5f;
	float spd_ki = 0.1f;
	float spd_kd = 0.0f;

	float pos_kp = 5.0f;
	float pos_ki = 0.0f;
	float pos_kd = 0.0f;

	bool using_abs_enc = false;
	UART_HandleTypeDef* abs_enc_uart = nullptr;//これやばい。一応AMT21xEncでnullptrは許容することにした
	size_t abs_enc_id = 0x54;
	float abs_gear_ratio = 1.0f;

	bool dob_enable = false;
	float dob_load_inertia = 1.0f;
	float dob_load_friction_coef = 0.0f;
	float dob_lpf_cutoff_freq = 5.0f;
	float dob_lpf_q_factor = 1.0f;
public:
	C6x0ControllerBuilder(size_t _motor_id,RobomasMD _m_type,MReg::ControlMode _c_mode = MReg::ControlMode::OPEN_LOOP,float _update_freq = 1000.0f){
		motor_id = _motor_id;
		m_type = _m_type;
		update_freq = _update_freq;
	}
	C6x0ControllerBuilder& set_default_speed_pid_gain(float kp,float ki,float kd){
		spd_kp = kp;
		spd_ki = ki;
		spd_kd = kd;
		return *this;
	}
	C6x0ControllerBuilder& set_default_position_pid_gain(float kp,float ki,float kd){
		pos_kp = kp;
		pos_ki = ki;
		pos_kd = kd;
		return *this;
	}
	C6x0ControllerBuilder& set_abs_enc_uart(float _using_abs_enc,UART_HandleTypeDef* _uart,size_t enc_id = 0x54,float gear_ratio = 1.0f){
		using_abs_enc = _using_abs_enc;
		abs_enc_uart = _uart;
		abs_enc_id = enc_id;
		abs_gear_ratio = gear_ratio;

		return *this;
	}
	C6x0ControllerBuilder& set_dob_param(bool _dob_enable,float _dob_load_inertia,float _dob_load_friction_coef,float _dob_lpf_cutoff_freq,float _dob_lpf_q_factor){
		dob_enable = _dob_enable;
		dob_load_inertia = _dob_load_inertia;
		dob_load_friction_coef = _dob_load_friction_coef;
		dob_lpf_cutoff_freq = _dob_lpf_cutoff_freq;
		dob_lpf_q_factor = _dob_lpf_q_factor;
		return *this;
	}

	C6x0Controller build(void){
		return C6x0Controller{
					motor_id,
					m_type,
					c_mode,
					dob_enable,
					using_abs_enc,

					CommonLib::Math::PIDBuilder(update_freq)
									.set_gain(spd_kp, spd_ki, spd_kd)
									.set_limit(1.0f)
									.build(),
					CommonLib::Math::PIDBuilder(update_freq)
									.set_gain(pos_kp, pos_ki, pos_kd)
									.set_limit(314.0f)
									.build(),
					CommonLib::Math::DisturbanceObserver<BoardLib::MotorInverceModel>(
										update_freq,
										BoardLib::MotorInverceModel(
											update_freq,
											dob_load_inertia,
											dob_load_friction_coef
										),
										dob_lpf_cutoff_freq,
										dob_lpf_q_factor
									),
					BoardLib::C6x0Enc(
									update_freq,
									RobomasMotorParam::get_gear_ratio(m_type)
									),
					BoardLib::AMT21xEnc(abs_enc_uart,0x54,update_freq,abs_gear_ratio)
		};
	}
};

}


#endif /* MOTOR_CONTROL_HPP_ */
