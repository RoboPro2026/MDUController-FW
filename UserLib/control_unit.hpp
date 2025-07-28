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
	CommonLib::IDMap id_map;

	CommonLib::GPIO led;
	CommonLib::Sequencer led_sequence;


	MotorUnit(void):
		id_map(CommonLib::IDMapBuilder()
				.add(MReg::MOTOR_STATE,    CommonLib::DataAccessor::generate<bool>(&is_active))
				.build()){
	}
};


}//namespace BoardLib



#endif /* CONTROL_UNIT_HPP_ */
