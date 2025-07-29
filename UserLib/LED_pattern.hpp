/*
 * LED_pattern.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef LED_PATTERN_HPP_
#define LED_PATTERN_HPP_


#include "CommonLib/sequencer.hpp"

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

	const CommonLib::Note* led_mode_indicate[2][3]={
			{BoardLib::LEDPattern::pwm_mode,BoardLib::LEDPattern::speed_mode,BoardLib::LEDPattern::position_mode},
			{BoardLib::LEDPattern::abs_pwm_mode,BoardLib::LEDPattern::abs_speed_mode,BoardLib::LEDPattern::abs_position_mode}
	};
}


#endif /* LED_PATTERN_HPP_ */
