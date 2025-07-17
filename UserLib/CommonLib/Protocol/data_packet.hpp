/*
 * data_packet.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_PROTOCOL_DATA_PACKET_HPP_
#define COMMONLIB_PROTOCOL_DATA_PACKET_HPP_

#include "id_defines.hpp"
#include "byte_reader_writer.hpp"

namespace CommonLib::Protocol{

	enum class IDBitPos:size_t{
		PRIORITY_BIT = 24,
		DATA_TYPE_BIT = 20,
		BOARD_ID_BIT = 16,
		REGISTER_ID_BIT = 0
	};

	struct DataPacket{
		bool is_request = false;
		size_t priority = 7;
		DataType data_type = DataType::COMMON_ID;
		size_t board_ID = 0;
		size_t register_ID = 0;

		uint8_t data[8] = {0};
		size_t data_length = 0;

		uint32_t generate_id(void) const {
			return ((priority&0xF) << static_cast<int>(IDBitPos::PRIORITY_BIT))
					| ((static_cast<size_t>(data_type)&0xF) << static_cast<size_t>(IDBitPos::DATA_TYPE_BIT))
					| ((board_ID&0xF) << static_cast<int>(IDBitPos::BOARD_ID_BIT))
					| (register_ID&0xFFFF);
		}
		void apply_id(uint32_t id){
			priority = (id >> static_cast<int>(IDBitPos::PRIORITY_BIT))&0xF;
			data_type = static_cast<Protocol::DataType>((id >> static_cast<size_t>(IDBitPos::DATA_TYPE_BIT))&0xF);
			board_ID = (id >> static_cast<int>(IDBitPos::BOARD_ID_BIT))&0xF;
			register_ID = id & 0xFFFF;
		}

		ByteWriter writer(void){
			return ByteWriter(data,sizeof(data),data_length);
		}
		ByteReader reader(void)const{
			return ByteReader(data,sizeof(data));
		}

	};
}



#endif /* COMMONLIB_PROTOCOL_DATA_PACKET_HPP_ */
