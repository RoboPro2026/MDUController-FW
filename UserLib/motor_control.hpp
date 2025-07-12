/*
 * motor_control.hpp
 *
 *  Created on: Jul 12, 2025
 *      Author: gomas
 */

#ifndef MOTOR_CONTROL_HPP_
#define MOTOR_CONTROL_HPP_

#include "CommonLib/pid.hpp"
#include "C6x0_encoder.hpp"
#include <optional>

namespace BoardLib{

enum class ControlMode:uint8_t{
	OPEN_LOOP,
	SPEED,
	POSITION,
//	ABS_POSITION,
};

class C6x0Controller{
private:
	const float motor_id;
	ControlMode mode;

	float power = 0.0f;
	float target_rad = 0.0f;
	float target_speed = 0.0f;
public:
	CommonLib::PIDController spd_pid;
	CommonLib::PIDController pos_pid;
	BoardLib::C6x0Enc enc;

	C6x0Controller(float _motor_id,float feedbuck_freq = 1000.0f,float _gear_ratio = 36.0f)
	:motor_id(_motor_id),
	mode(ControlMode::OPEN_LOOP),
	spd_pid(CommonLib::PIDBuilder(feedbuck_freq)
				.set_gain(0.000'1f, 0.000'05f, 0.0f)
				.set_limit(1.0f)
				.build()),
	pos_pid(CommonLib::PIDBuilder(feedbuck_freq)
				.set_gain(0.000'1f, 0.000'05f, 0.0f)
				.set_limit(0.0f)
				.build()),
	enc(feedbuck_freq,_gear_ratio){
	}

	void set_control_mode(ControlMode m);
	ControlMode get_control_mode(void)const{ return mode; }

	void set_target_speed_rad(float _target_speed){ target_speed = _target_speed;}
	float get_target_speed_rad(void) const {return target_speed;}

	void set_target_rad(float _target_rad){ target_rad = _target_rad;}
	float get_target_rad(void) const {return target_rad;}

	float pid_operation(const CommonLib::CanFrame &frame);

};
}


#endif /* MOTOR_CONTROL_HPP_ */
