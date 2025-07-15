/*
 * motor_control.hpp
 *
 *  Created on: Jul 12, 2025
 *      Author: gomas
 */

#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

#include "CommonLib/Math/pid.hpp"
#include "CommonLib/Math/disturbance_observer.hpp"
#include "motor_model.hpp"
#include "C6x0_encoder.hpp"
#include <optional>

namespace BoardLib{

enum class ControlMode:uint8_t{
	OPEN_LOOP,
	SPEED,
	POSITION,
//	ABS_POSITION,
};

enum class MotorType:uint8_t{
	C610 = 0x00,
	C620 = 0x01,
	VESC = 0x10,
};

class C6x0Controller{
private:
	//ロボマスモーターのモーターパラメータ
	//TODO:値の確認
	static constexpr float torque_coef[2] = {0.18, 0.3};
	static constexpr float gear_ratio[2] = {19.0,36.0};
	static constexpr float get_gear_ratio(MotorType m_type){
		return m_type != MotorType::VESC ? gear_ratio[static_cast<size_t>(m_type)] : gear_ratio[static_cast<size_t>(MotorType::C610)];
	}
	static constexpr float get_torque_coef(MotorType m_type){
		return m_type != MotorType::VESC ? gear_ratio[static_cast<size_t>(m_type)] : torque_coef[static_cast<size_t>(MotorType::C610)];
	}

	const float motor_id;
	ControlMode mode;
	MotorType motor_type;
	bool dob_en;

	float power = 0.0f;
	float target_rad = 0.0f;
	float target_speed = 0.0f;
public:
	CommonLib::Math::PIDController spd_pid;
	CommonLib::Math::PIDController pos_pid;
	CommonLib::Math::DisturbanceObserver<BoardLib::MotorInverceModel> dob;
	BoardLib::C6x0Enc enc;

	C6x0Controller(float _motor_id,float feedbuck_freq = 1000.0f,MotorType m_type = MotorType::C610,bool _dob_en = false,float _dob_cutoff = 5.0f)
	:motor_id(_motor_id),
	mode(ControlMode::OPEN_LOOP),
	dob_en(_dob_en),
	spd_pid(CommonLib::Math::PIDBuilder(feedbuck_freq)
				.set_gain(0.000'1f, 0.000'05f, 0.0f)
				.set_limit(1.0f)
				.build()),
	pos_pid(CommonLib::Math::PIDBuilder(feedbuck_freq)
				.set_gain(0.000'1f, 0.000'05f, 0.0f)
				.set_limit(0.0f)
				.build()),
	dob(feedbuck_freq,BoardLib::MotorInverceModel(feedbuck_freq,1.0,1.0),_dob_cutoff),
	enc(feedbuck_freq,get_gear_ratio(m_type)){
	}

	void set_motor_type(MotorType m_type){
		motor_type = m_type;
		enc.set_gear_ratio(get_gear_ratio(m_type));
	}

	void set_control_mode(ControlMode m);
	ControlMode get_control_mode(void)const{ return mode; }

	void set_target_speed_rad(float _target_speed){ target_speed = _target_speed;}
	float get_target_speed_rad(void) const {return target_speed;}

	void set_target_rad(float _target_rad){ target_rad = _target_rad;}
	float get_target_rad(void) const {return target_rad;}

	float pid_operation(const CommonLib::CanFrame &frame);
	float get_power(void) const {return power;}

};
}


#endif /* MOTOR_CONTROL_HPP_ */
