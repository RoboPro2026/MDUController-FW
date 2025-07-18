/*
 * motor_control.cpp
 *
 *  Created on: Jul 12, 2025
 *      Author: gomas
 */


#include "motor_control.hpp"

namespace BoardLib{
void C6x0Controller::set_control_mode(MReg::ControlMode _mode){
	if(abs_enc.is_dead()){
		using_abs_enc = false;
	}
	float rad = using_abs_enc ? abs_enc.get_rad(): enc.get_rad();

	switch(_mode){
	case MReg::ControlMode::POSITION:
		target_rad = rad;
		target_rad = 0.0f;
		target_speed = 0.0f;
		spd_pid.reset();
		pos_pid.reset();
		break;
	default:
		torque= 0.0f;
		target_rad = 0.0f;
		target_speed = 0.0f;
		spd_pid.reset();
		pos_pid.reset();
		break;
	}
	dob.reset();
	mode = _mode;
}

float C6x0Controller::pid_operation(const CommonLib::CanFrame &frame){
	enc.update_by_can_msg(frame);
	if(abs_enc.is_dead()){
		using_abs_enc = false;
	}
	float rad = using_abs_enc ? abs_enc.get_rad(): enc.get_rad();
	float rad_spd = using_abs_enc ? abs_enc.get_rad_speed(): enc.get_rad_speed();

	switch(mode){
	case MReg::ControlMode::POSITION:
		target_speed = pos_pid(target_rad,rad);
	case MReg::ControlMode::SPEED:
		torque = spd_pid(target_speed,rad_spd);
		if(dob_enable){
			torque -= dob.observe_disturbance(rad_spd,torque);
		}
	case MReg::ControlMode::OPEN_LOOP:
		//nop
		break;
	default:
		//nop
		break;
	}

	abs_enc.request_position();
	return torque;
}
}

