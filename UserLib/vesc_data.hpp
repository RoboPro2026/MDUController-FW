/*
 * vesc_data.hpp
 *
 *  Created on: Jul 21, 2025
 *      Author: gomas
 */

#ifndef VESC_DATA_HPP_
#define VESC_DATA_HPP_

#include "CommonLib/can_if.hpp"
#include <cmath>
#include <cassert>
#include <optional>

namespace BoardLib{

//TODO:制作
class VescDataConverter{
	static constexpr uint32_t vesc_duty_id = 0x0000'0000;
	static constexpr uint32_t vesc_current_id = 0x0000'0100;
	static constexpr uint32_t vesc_rpm_id = 0x0000'0300;
	static constexpr uint32_t vesc_position_id = 0x0000'0400;

	static constexpr float duty_coef = 100'000.0f;
	static constexpr float current_coef = 1000.0f;
	static constexpr float rad_spd_to_rpm_coef = 60.0f*1000.0f/(2*M_PI);
	static constexpr float position_coef = 360.0f/(2*M_PI);

	static std::optional<CommonLib::CanFrame> generate_frame(size_t m_id,MReg::VescMode m,float value){

		CommonLib::CanFrame cf;
		cf.is_ext_id = true;
		auto writer = cf.writer();
		float converted_value;
		switch(m){
		case MReg::VescMode::NOP:
			return std::nullopt;
		case MReg::VescMode::PWM:
			cf.id = vesc_duty_id | m_id;
			converted_value = value * duty_coef;
			writer.write<int32_t>(static_cast<int32_t>(converted_value),false);
			return cf;
		case MReg::VescMode::CURRENT:
			cf.id = vesc_current_id | m_id;
			converted_value = value * current_coef;
			break;
		case MReg::VescMode::SPEED:
			cf.id = vesc_rpm_id | m_id;
			converted_value = value * rad_spd_to_rpm_coef;
			break;
		case MReg::VescMode::POSITION:
			cf.id = vesc_position_id | m_id;
			converted_value = value * position_coef;
			break;
		}
		writer.write<int32_t>(static_cast<int32_t>(converted_value),false);
		return cf;
	}

	const size_t motor_id;
	MReg::VescMode mode;
public:
	VescDataConverter(size_t _motor_id,MReg::VescMode _mode = MReg::VescMode::NOP)
	:motor_id(_motor_id),
	 mode(_mode){
	}

	void set_mode(MReg::VescMode m){
		mode = m;
	}
	MReg::VescMode get_mode(void)const{
		return mode;
	}

	std::optional<CommonLib::CanFrame> generate_frame(float value){
		return generate_frame(motor_id,mode,value);
	}
};
}


#endif /* VESC_DATA_HPP_ */
