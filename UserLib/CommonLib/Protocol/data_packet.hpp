/*
 * data_packet.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_PROTOCOL_DATA_PACKET_HPP_
#define COMMONLIB_PROTOCOL_DATA_PACKET_HPP_

#include "byte_reader_writer.hpp"

namespace SabaneLib::Protocol{
	enum class DataType : uint8_t{
		COMMON_DATA,
		PCU_DATA,
		RMC_DATA,
		GPIOC_DATA,
		COMMON_DATA_ENFORCE = 0xF
	};

	struct DataPacket{
		bool is_request = false;
		uint8_t priority = 7;
		DataType data_type = DataType::COMMON_DATA;
		uint8_t board_ID = 0;
		uint16_t register_ID = 0;

		uint8_t data[8] = {0};
		size_t data_length = 0;

		ByteWriter writer(void){
			return ByteWriter(data,sizeof(data),data_length);
		}
		ByteReader reader(void)const{
			return ByteReader(data,sizeof(data));
		}

	};
}



#endif /* COMMONLIB_PROTOCOL_DATA_PACKET_HPP_ */
