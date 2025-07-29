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
#include "motor_param.hpp"
#include "motor_calibration.hpp"
#include "abs_encoder.hpp"

#include <optional>
#include <memory>

namespace BoardLib{

class C6x0Controller{
private:
	const size_t motor_id; //0~3
	MReg::RobomasMD motor_type;
	MReg::ControlMode mode;
	bool dob_enable;
	bool using_abs_enc;

	float torque = 0.0f;
	float target_rad = 0.0f;
	float target_speed = 0.0f;

	float rad_origin = 0.0f;

	bool estimate_motor_type_f = true;
	bool calibration_request = false;

public:
	CalibrationManager calib_mng;
	CommonLib::Math::PIDController spd_pid;
	CommonLib::Math::PIDController pos_pid;
	CommonLib::Math::DisturbanceObserver<BoardLib::MotorInverceModel> dob;
	BoardLib::C6x0Enc enc;
	std::unique_ptr<BoardLib::IABSEncoder> abs_enc;

	C6x0Controller(
			size_t _motor_id,
			MReg::RobomasMD _m_type,
			MReg::ControlMode _mode,
			bool _dob_en,
			bool _using_abs_enc,
			BoardLib::CalibrationManager&& _calib_mng,
			CommonLib::Math::PIDController&& _spd_pid,
			CommonLib::Math::PIDController&& _pos_pid,
			CommonLib::Math::DisturbanceObserver<BoardLib::MotorInverceModel>&& _dob,
			BoardLib::C6x0Enc&& _enc,
			std::unique_ptr<BoardLib::IABSEncoder> _abs_enc)
	:motor_id(_motor_id),
	 motor_type(_m_type),
	 mode(_mode),
	 dob_enable(_dob_en),
	 using_abs_enc(_using_abs_enc),
	 calib_mng(std::move(_calib_mng)),
	 spd_pid(std::move(_spd_pid)),
	 pos_pid(std::move(_pos_pid)),
	 dob(std::move(_dob)),
	 enc(std::move(_enc)),
	 abs_enc(std::move(_abs_enc)){
	}
	size_t get_motor_id(void)const{return motor_id;}

	void set_motor_type(MReg::RobomasMD m_type){
		motor_type = m_type;
		enc.set_motor_type(m_type);
	}
	MReg::RobomasMD get_motor_type(void) const {return motor_type;}
	void estimate_motor_type(bool enable = true){estimate_motor_type_f = enable;}

	void set_control_mode(MReg::ControlMode _mode);
	MReg::ControlMode get_control_mode(void)const{ return mode; }

	void use_abs_enc(bool _using_abs_enc = true){ using_abs_enc = abs_enc->is_dead() ? false : _using_abs_enc; }
	bool is_using_abs_enc(void)const{return using_abs_enc;}

	void use_dob(bool dob_en = true){dob_enable = dob_en;}
	bool is_using_dob(void)const {return dob_enable;}

	void  set_torque(float _torque){torque = _torque;}
	float get_torque(void) const {return torque;}

	void set_target_speed_rad(float _target_speed){ target_speed = _target_speed;}
	float get_target_speed_rad(void) const {return target_speed;}

	void set_target_rad(float _target_rad){ target_rad = _target_rad + rad_origin;}
	float get_target_rad(void) const {return target_rad - rad_origin;}

	void overwrite_rad(float rad){rad_origin = enc.get_rad() - rad;}
	float get_overwrited_rad(void)const{return enc.get_rad() - rad_origin;}

	void start_calibration(void){calibration_request = true;}
	bool is_calibrating(void)const{return calibration_request;}

	//CANの受信割込みで呼び出し
	bool update(const CommonLib::CanFrame &frame);
	int16_t get_current_can_format(void)const{
		return RobomasMotorParam::torque_to_robomas_value(motor_type, torque);
	}
};

inline void C6x0Controller::set_control_mode(MReg::ControlMode _mode){
	if(abs_enc){
		if(abs_enc->is_dead()){
			using_abs_enc = false;
		}
	}else{
		using_abs_enc = false;
	}

	target_rad = using_abs_enc ? abs_enc->get_rad(): enc.get_rad();
	target_rad = 0.0f;
	target_speed = 0.0f;
	spd_pid.reset();
	pos_pid.reset();
	dob.reset();
	mode = _mode;
}

inline bool C6x0Controller::update(const CommonLib::CanFrame &frame){
	if(frame.id != (0x201 + motor_id)){
		return false;
	}

	if(estimate_motor_type_f){
		if(frame.data[6] == 0){
			set_motor_type(MReg::RobomasMD::C610);
		}else{
			set_motor_type(MReg::RobomasMD::C620);
		}
		estimate_motor_type_f = false;
	}

	enc.update_by_can_msg(frame);

	if(calibration_request){
		auto [_torque,_continue] = calib_mng.calibration(enc.get_rad_speed(),enc.get_torque());
		calibration_request = _continue;
		torque = _torque;
		if(not calibration_request && not calib_mng.is_failed()){
			dob.inverse_model.set_inertia(calib_mng.get_inertia());
			dob.inverse_model.set_friction_coef(calib_mng.get_friction_coef());
		}
		return true;
	}

	if(abs_enc){
		if(abs_enc->is_dead()){
			using_abs_enc = false;
		}
	}else{
		using_abs_enc = false;
	}

	float rad = using_abs_enc ? abs_enc->get_rad(): enc.get_rad();
	float rad_spd = using_abs_enc ? abs_enc->get_rad_speed(): enc.get_rad_speed();

	switch(mode){
	case MReg::ControlMode::POSITION:
		target_speed = pos_pid(target_rad,rad);
	case MReg::ControlMode::SPEED:
		torque = spd_pid(target_speed,rad_spd);
		if(dob_enable){
			torque -= dob.observe_disturbance(rad_spd,torque);
		}
		break;
	default:
		break;
	}
	return true;
}


///////////////////////////////////////////////////////////////////////////////
//builder
///////////////////////////////////////////////////////////////////////////////

class C6x0ControllerBuilder{
private:
	size_t motor_id = 0;
	MReg::RobomasMD m_type = MReg::RobomasMD::C610;
	MReg::ControlMode c_mode = MReg::ControlMode::OPEN_LOOP;
	float update_freq = 1000.0f;

	float spd_kp = 0.5f;
	float spd_ki = 0.0f;//0.1f;
	float spd_kd = 0.0f;
	float trq_limit = 1.0f;

	float pos_kp = 5.0f;
	float pos_ki = 0.0f;
	float pos_kd = 0.0f;
	float spd_limit = 314.0f;

	bool using_abs_enc = false;
	std::unique_ptr<IABSEncoder> abs_enc = nullptr;

	bool dob_enable = false;
	float dob_load_inertia = 0.0004f; //直径100mm,300gの円盤のイナーシャ
	float dob_load_friction_coef = 0.0005f;
	float dob_lpf_cutoff_freq = 5.0f;
	float dob_lpf_q_factor = 1.0f;

	int calib_measurement_n = 6;
	float calib_on_torque = 0.1f; //TODO:なんか0.1とかにするとシミュできない
	float calib_settling_time = 5.0f;
public:
	C6x0ControllerBuilder(size_t _motor_id,MReg::RobomasMD _m_type,MReg::ControlMode _c_mode = MReg::ControlMode::OPEN_LOOP,float _update_freq = 1000.0f){
		motor_id = _motor_id;
		m_type = _m_type;
		update_freq = _update_freq;
	}
	C6x0ControllerBuilder& set_default_speed_pid_gain(float kp,float ki,float kd,float trq_lim){
		spd_kp = kp;
		spd_ki = ki;
		spd_kd = kd;
		trq_limit = trq_lim;
		return *this;
	}
	C6x0ControllerBuilder& set_default_position_pid_gain(float kp,float ki,float kd,float spd_lim){
		pos_kp = kp;
		pos_ki = ki;
		pos_kd = kd;
		spd_limit = spd_lim;
		return *this;
	}
	C6x0ControllerBuilder& set_abs_enc(std::unique_ptr<IABSEncoder> _abs_enc,bool _using_abs_enc){
		using_abs_enc = _using_abs_enc;
		abs_enc = std::move(_abs_enc);
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
	C6x0ControllerBuilder& set_calibration_param(int _measurement_n,float _on_torque = 0.1f,float _settling_time = 5.0f){
		calib_measurement_n = _measurement_n;
		calib_on_torque = _on_torque;
		calib_settling_time = _settling_time;
		return *this;
	}

	C6x0Controller build(void){
		return C6x0Controller{
					motor_id,
					m_type,
					c_mode,
					dob_enable,
					using_abs_enc,
					BoardLib::CalibrationManager(
							calib_measurement_n,
							update_freq,
							calib_on_torque,
							calib_settling_time),
					CommonLib::Math::PIDBuilder(update_freq)
									.set_gain(spd_kp, spd_ki, spd_kd)
									.set_limit(trq_limit)
									.build(),
					CommonLib::Math::PIDBuilder(update_freq)
									.set_gain(pos_kp, pos_ki, pos_kd)
									.set_limit(spd_limit)
									.build(),
					CommonLib::Math::DisturbanceObserver<BoardLib::MotorInverceModel>(
							update_freq,
							BoardLib::MotorInverceModel(
								update_freq,
								dob_load_inertia,
								dob_load_friction_coef
							),
							dob_lpf_cutoff_freq,
							dob_lpf_q_factor),
					BoardLib::C6x0Enc(m_type,update_freq),
					std::move(abs_enc)
		};
	}
};

}


#endif /* MOTOR_CONTROL_HPP_ */
