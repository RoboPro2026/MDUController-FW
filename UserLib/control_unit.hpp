/*
 * control_unit.hpp
 *
 *  Created on: Jul 28, 2025
 *      Author: gomas
 */

#ifndef CONTROL_UNIT_HPP_
#define CONTROL_UNIT_HPP_

#include "CommonLib/Protocol/id_defines.hpp"
#include "CommonLib/id_map_control.hpp"
#include "CommonLib/gpio.hpp"
#include "CommonLib/sequencer.hpp"

#include "motor_control.hpp"
#include "vesc_data.hpp"

namespace BoardLib{
struct MotorUnit{
	bool is_active = false;
	C6x0Controller rm_motor;
	VescDataConverter vesc_motor;

	CommonLib::GPIO led;
	CommonLib::Sequencer led_sequence;

	CommonLib::IDMap id_map;

	void set_control_mode(uint8_t c_val){

	}

	MotorUnit(int id,GPIO_TypeDef *led_port,uint_fast16_t led_pin):
		rm_motor(C6x0ControllerBuilder(id,MReg::RobomasMD::C610).build()),
		vesc_motor(id),
		led(led_port,led_pin),
		led_sequence([&](float v){led(v>0.0f);}),
		id_map(CommonLib::IDMapBuilder()
				.add(MReg::MOTOR_STATE,    CommonLib::DataAccessor::generate<bool>(&is_active))
				.add(MReg::CONTROL,        CommonLib::DataAccessor::generate<uint8_t>([&](uint8_t v)mutable{set_control_mode(v);}))
				.add(MReg::ABS_GEAR_RATIO, CommonLib::DataAccessor::generate<bool>(&is_active))
				.add(MReg::CAL_RQ,         CommonLib::DataAccessor::generate<bool>(&is_active))
				.add(MReg::LOAD_J,         CommonLib::DataAccessor::generate<float>([&](float j)mutable{rm_motor.dob.inverse_model.set_inertia(j);}))
				.build()){
	}
};


}//namespace BoardLib



#endif /* CONTROL_UNIT_HPP_ */
