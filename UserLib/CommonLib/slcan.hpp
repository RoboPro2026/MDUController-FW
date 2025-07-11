/*
 * slcan.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_SLCAN_HPP_
#define COMMONLIB_SLCAN_HPP_

#include "can_if.hpp"

namespace SabaneLib::SLCAN{
	constexpr size_t SLCAN_STR_MAX_SIZE = 28;
	size_t can_to_slcan(const CanFrame &frame,char *str,const size_t str_max_size);
	bool slcan_to_can(const char *str, CanFrame &frame);
}

#endif /* COMMONLIB_SLCAN_HPP_ */
