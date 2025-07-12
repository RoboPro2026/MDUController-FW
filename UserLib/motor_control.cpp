/*
 * motor_control.cpp
 *
 *  Created on: Jul 12, 2025
 *      Author: gomas
 */


#include "motor_control.hpp"

namespace BoardLib{
void C6x0Controller::set_control_mode(ControlMode _mode){
	switch(_mode){
	case ControlMode::POSITION:
		target_rad = enc.get_rad();
		target_rad = 0.0f;
		target_speed = 0.0f;
		spd_pid.reset();
		pos_pid.reset();
		break;
//	case ControlMode::ABS_POSITION:
//		target_rad = abs_state.rad;
//		target_speed = 0.0f;
//		power = 0.0f;
//		spd_pid.reset();
//		pos_pid.reset();
//		break;
	default:
		power = 0.0f;
		target_rad = 0.0f;
		target_speed = 0.0f;
		spd_pid.reset();
		pos_pid.reset();
		break;
	}
	mode = _mode;
}

float C6x0Controller::pid_operation(const CommonLib::CanFrame &frame){
	enc.update_by_can_msg(frame);

	switch(mode){
	case ControlMode::OPEN_LOOP:
		//nop
		break;
	case ControlMode::SPEED:
		power = spd_pid(target_speed,enc.get_rad_speed());
		break;
	case ControlMode::POSITION:
		target_speed = pos_pid(target_rad,enc.get_rad());
		power = spd_pid(target_speed,enc.get_rad_speed());
		break;
//	case ControlMode::ABS_POSITION_MODE:
//		//nop
//		break;
	default:
		//nop
		break;
	}

	return power;
}
}

