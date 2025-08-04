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

static std::optional<CommonLib::CanFrame> generate_vesc_frame(size_t m_id,MReg::VescMode m,float value){
	constexpr uint32_t vesc_duty_id = 0x0000'0000;
	constexpr uint32_t vesc_current_id = 0x0000'0100;
	constexpr uint32_t vesc_rpm_id = 0x0000'0300;
	constexpr uint32_t vesc_position_id = 0x0000'0400;

	constexpr float duty_coef = 100'000.0f;
	constexpr float current_coef = 1000.0f;
	constexpr float rad_spd_to_rpm_coef = 60.0f*1000.0f/(2*M_PI);
	constexpr float position_coef = 360.0f/(2*M_PI); //間違ってるかも

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
		writer.write<int32_t>(static_cast<int32_t>(converted_value),false);
		return cf;
	case MReg::VescMode::SPEED:
		cf.id = vesc_rpm_id | m_id;
		converted_value = value * rad_spd_to_rpm_coef;
		writer.write<int32_t>(static_cast<int32_t>(converted_value),false);
		return cf;
	case MReg::VescMode::POSITION:
		cf.id = vesc_position_id | m_id;
		converted_value = value * position_coef;
		writer.write<int32_t>(static_cast<int32_t>(converted_value),false);
		return cf;
	}
	return std::nullopt;
}

class VescController{
	const size_t motor_id;
	MReg::VescMode mode;
	float value;
public:
	VescController(size_t _motor_id,MReg::VescMode _mode = MReg::VescMode::NOP)
	:motor_id(_motor_id),
	 mode(_mode),
	 value(0.0f){
	}

	void set_mode(MReg::VescMode m){
		mode = m;
	}
	MReg::VescMode get_mode(void)const{
		return mode;
	}
	void set_value(float v){
		value = v;
	}
	float get_value(void)const{
		return value;
	}

	std::optional<CommonLib::CanFrame> get_frame(void){
		return generate_vesc_frame(motor_id,mode,value);
	}
};
}


#endif /* VESC_DATA_HPP_ */
