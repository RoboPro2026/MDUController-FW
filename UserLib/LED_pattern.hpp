/*
 * LED_pattern.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef LED_PATTERN_HPP_
#define LED_PATTERN_HPP_


#include "CommonLib/sequencable_io.hpp"

namespace BoardLib::LEDPattern{

	inline constexpr CommonLib::Note ok[] = {
		{1.0f,10},
		{0.0f,10},
		CommonLib::end_of_io_sequence
	};
	inline constexpr CommonLib::Note setting[]={
		{1.0f,100},
		{0.0f,100},
		{1.0f,700},
		{0.0f,100},
		CommonLib::end_of_io_sequence
	};

	inline constexpr CommonLib::Note test[]={
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,100},

		{1.0f,500},
		{0.0f,100},
		{1.0f,500},
		{0.0f,100},
		{1.0f,500},
		{0.0f,100},

		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,1000},
		CommonLib::end_of_io_sequence
	};

	inline const CommonLib::Note pwm_mode[] = {
		{1.0,100},
		{0.0,1900},
		CommonLib::end_of_io_sequence
	};
	inline const CommonLib::Note speed_mode[] = {
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,1700},

		CommonLib::end_of_io_sequence
	};
	inline const CommonLib::Note position_mode[] = {
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,1500},

		CommonLib::end_of_io_sequence
	};

	inline const CommonLib::Note abs_pwm_mode[] = {
		{1.0,500},
		{0.0,100},
		{1.0,100},
		{0.0,1300},
		CommonLib::end_of_io_sequence
	};
	inline const CommonLib::Note abs_speed_mode[] = {
		{1.0f,500},
		{0.0f,100},
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,1100},

		CommonLib::end_of_io_sequence
	};
	inline const CommonLib::Note abs_position_mode[] = {
		{1.0f,500},
		{0.0f,100},
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,100},
		{1.0f,100},
		{0.0f,900},

		CommonLib::end_of_io_sequence
	};
}


#endif /* LED_PATTERN_HPP_ */
