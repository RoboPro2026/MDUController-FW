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

	inline constexpr SabaneLib::Note ok[] = {
		{1.0f,10},
		{0.0f,10},
		SabaneLib::end_of_io_sequence
	};
	inline constexpr SabaneLib::Note setting[]={
		{1.0f,100},
		{0.0f,100},
		{1.0f,700},
		{0.0f,100},
		SabaneLib::end_of_io_sequence
	};
}


#endif /* LED_PATTERN_HPP_ */
