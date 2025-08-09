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
		{1.0f,100},
		{0.0f,100},
		CommonLib::end_of_io_sequence
	};
	inline constexpr CommonLib::Note running[] = {
		{0.2f,100},
		{0.0f,900},
		CommonLib::end_of_io_sequence
	};
	inline constexpr CommonLib::Note error[]={
		{1.0f,100},
		{0.0f,100},
		{1.0f,700},
		{0.0f,100},
		CommonLib::end_of_io_sequence
	};

	inline constexpr CommonLib::Note sos[]={
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

	inline const CommonLib::Note calibrating[] = {
			{1.0f,100},
			{0.0f,100},
			CommonLib::end_of_io_sequence
	};

	inline const CommonLib::Note pwm_mode[] = {
		{0.0,  1},
		{1.0,100},
		{0.0,1899},
		CommonLib::end_of_io_sequence
	};
	inline const CommonLib::Note speed_mode[] = {
		{0.0f,  1},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,1599},

		CommonLib::end_of_io_sequence
	};
	inline const CommonLib::Note position_mode[] = {
		{0.0f,1},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,1299},

		CommonLib::end_of_io_sequence
	};

	inline const CommonLib::Note abs_pwm_mode[] = {
		{0.0f,1},
		{1.0,500},
		{0.0,200},
		{1.0,100},
		{0.0,1299},
		CommonLib::end_of_io_sequence
	};
	inline const CommonLib::Note abs_speed_mode[] = {
		{0.0f,  1},
		{1.0f,500},
		{0.0f,200},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,899},

		CommonLib::end_of_io_sequence
	};
	inline const CommonLib::Note abs_position_mode[] = {
		{0.0f,  1},
		{1.0f,500},
		{0.0f,200},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,200},
		{1.0f,100},
		{0.0f,599},

		CommonLib::end_of_io_sequence
	};

	inline const CommonLib::Note* led_mode_indicate[2][3]={
			{BoardLib::LEDPattern::pwm_mode,BoardLib::LEDPattern::speed_mode,BoardLib::LEDPattern::position_mode},
			{BoardLib::LEDPattern::abs_pwm_mode,BoardLib::LEDPattern::abs_speed_mode,BoardLib::LEDPattern::abs_position_mode}
	};

	inline const CommonLib::Note vesc_only[] = {
			{0.0f,  1},
			{1.0f,1000},
			{0.0f,999},

			CommonLib::end_of_io_sequence
		};
}


#endif /* LED_PATTERN_HPP_ */
