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

namespace BoardLib{

enum class VescMode{
	CURRENT,
	SPEED,
	POSITION,
	PWM
};

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

	static CommonLib::CanFrame generate_frame(size_t m_id,VescMode m,float value){
		CommonLib::CanFrame cf;
		cf.is_ext_id = true;
		auto writer = cf.writer();
		float converted_value;
		switch(m){
		case VescMode::CURRENT:
			cf.id = vesc_current_id | m_id;
			converted_value = value * current_coef;
			break;
		case VescMode::SPEED:
			cf.id = vesc_rpm_id | m_id;
			converted_value = value * rad_spd_to_rpm_coef;
			break;
		case VescMode::POSITION:
			cf.id = vesc_position_id | m_id;
			converted_value = value * position_coef;
			break;
		case VescMode::PWM:
			cf.id = vesc_duty_id | m_id;
			converted_value = value * duty_coef;
			break;
		}
		writer.write<int32_t>(static_cast<int32_t>(converted_value),false);
		return cf;
	}

	const size_t motor_id;
	VescMode mode;
public:
	VescDataConverter(size_t _motor_id,VescMode _mode = VescMode::PWM)
	:motor_id(_motor_id),
	 mode(_mode){
	}

	void set_mode(VescMode m){
		mode = m;
	}
	VescMode get_mode(void)const{
		return mode;
	}

	CommonLib::CanFrame generate_frame(float value){
		return generate_frame(motor_id,mode,value);
	}

	CommonLib::CanFrame generate_set_duty_frame(float duty){
		assert(-1.0f<duty && duty<1.0f);
		return generate_frame(motor_id,mode,duty);
	}
	CommonLib::CanFrame generate_set_current_frame(float current_A){
		return generate_frame(motor_id,mode,current_A);
	}
	CommonLib::CanFrame generate_set_speed_frame(float speed_rad){
		return generate_frame(motor_id,mode,speed_rad);
	}
	CommonLib::CanFrame generate_set_position_frame(float position_rad){
		return generate_frame(motor_id,mode,position_rad);
	}





};
}


#endif /* VESC_DATA_HPP_ */
