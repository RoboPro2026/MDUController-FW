/*
 * motor_control.cpp
 *
 *  Created on: Jul 12, 2025
 *      Author: gomas
 */


#include "motor_control.hpp"

namespace BoardLib{
void C6x0Controller::set_control_mode(MReg::ControlMode _mode){
	switch(_mode){
	case MReg::ControlMode::POSITION:
		target_rad = enc.get_rad();
		target_rad = 0.0f;
		target_speed = 0.0f;
		spd_pid.reset();
		pos_pid.reset();
		break;
	case MReg::ControlMode::ABS_POSITION:
		target_rad = abs_enc.get_rad();
		target_speed = 0.0f;
		torque = 0.0f;
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

float C6x0Controller::pid_operation(const CommonLib::CanFrame &frame){ //
	enc.update_by_can_msg(frame);

	switch(mode){
	case MReg::ControlMode::POSITION:
		target_speed = pos_pid(target_rad,enc.get_rad());
	case MReg::ControlMode::SPEED:
		torque = spd_pid(target_speed,enc.get_rad_speed());
		if(dob_enable){
			torque -= dob.observe_disturbance(enc.get_rad_speed(),torque);
		}
	case MReg::ControlMode::OPEN_LOOP:
		//nop
		break;
	case  MReg::ControlMode::ABS_POSITION:
		if(abs_enc.is_dead()){
			set_control_mode(MReg::ControlMode::OPEN_LOOP);
			break;
		}
		target_speed = pos_pid(target_rad,abs_enc.get_rad());
		torque = spd_pid(target_speed,abs_enc.get_rad_speed());
		if(dob_enable){
			torque -= dob.observe_disturbance(abs_enc.get_rad_speed(),torque);
		}
		break;
	default:
		//nop
		break;
	}

	abs_enc.request_position();
	return torque;
}
}

