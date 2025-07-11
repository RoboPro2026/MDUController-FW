/*
 * can.hpp
 *
 *  Created on: Sep 26, 2024
 *      Author: gomas
 */

#ifndef CAN_HPP_
#define CAN_HPP_

#include "main.h"

#include "ring_buffer.hpp"

#include "Protocol/byte_reader_writer.hpp"
#include "Protocol/data_packet.hpp"

#include <optional>

namespace SabaneLib{
	struct CanFrame{
		uint8_t data[8]={0};
		size_t data_length=0;
		uint32_t id=0;
		bool is_ext_id=false;
		bool is_remote=false;

		CanFrame(void){}
		CanFrame(Protocol::DataPacket data){
			decode_common_data_packet(data);
		}

		Protocol::ByteWriter writer(void){
			return Protocol::ByteWriter(data,sizeof(data), data_length);
		}
		Protocol::ByteReader reader(void)const{
			return Protocol::ByteReader(data,sizeof(data));
		}

		enum class CanIDBitPos{
			PRIORITY_BIT = 24,
			DATA_TYPE_BIT = 20,
			BOARD_ID_BIT = 16,
			REGISTER_ID_BIT = 0
		};

		std::optional<Protocol::DataPacket> to_common_data_packet(void){
			if(not is_ext_id){
				return std::nullopt;
			}

			Protocol::DataPacket dp;
			dp.is_request = is_remote;
			dp.priority = (id>> static_cast<int>(CanIDBitPos::PRIORITY_BIT))&0xF;
			dp.data_type = static_cast<Protocol::DataType>((id >> static_cast<int>(CanIDBitPos::DATA_TYPE_BIT))&0xF);
			dp.board_ID = (id >> static_cast<int>(CanIDBitPos::BOARD_ID_BIT))&0xF;
			dp.register_ID = id & 0xFFFF;

			memcpy(dp.data, data,data_length);
			dp.data_length = data_length;

			return dp;
		}

		void decode_common_data_packet(const Protocol::DataPacket &dp){
			is_ext_id = true;
			is_remote = dp.is_request;

			id = ((dp.priority&0xF)<<static_cast<int>(CanIDBitPos::PRIORITY_BIT))
					| ((static_cast<uint8_t>(dp.data_type)&0xF) << static_cast<int>(CanIDBitPos::DATA_TYPE_BIT) )
					| ((dp.board_ID&0xF) << static_cast<int>(CanIDBitPos::BOARD_ID_BIT))
					| (dp.register_ID&0xFFFF);

			data_length = dp.data_length;
			memcpy(data, dp.data,dp.data_length);
		}

	};

	enum class CanFilterMode{
		ONLY_STD,
		ONLY_EXT,
		STD_AND_EXT,
	};

	class ICan{
	public:
		uint32_t virtual tx_available(void)const = 0;
		bool virtual tx(const CanFrame &tx_frame) = 0;

		uint32_t virtual rx_available(void)const = 0;
		bool virtual rx(CanFrame &rx_frame) = 0;

		virtual ~ICan(){}
	};
}



#endif /* CAN_HPP_ */
