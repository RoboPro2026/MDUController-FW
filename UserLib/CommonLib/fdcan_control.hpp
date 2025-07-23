/*
 * fdcan_control.hpp
 *
 *  Created on: Sep 26, 2024
 *      Author: gomas
 */

#ifndef FDCAN_CONTROL_HPP_
#define FDCAN_CONTROL_HPP_

#include "main.h"

#ifdef HAL_FDCAN_MODULE_ENABLED

#include "can_if.hpp"

#include <memory>
#include <cassert>
#include <optional>

namespace CommonLib{

	struct FdCanRxFifoParams{
		uint32_t no;
		uint32_t filter;
		uint32_t it;
		uint32_t it_buff;
	};

	constexpr FdCanRxFifoParams FdCanRxFifo0 = {
			FDCAN_RX_FIFO0,
			FDCAN_FILTER_TO_RXFIFO0,
			FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
			FDCAN_FLAG_RX_FIFO0_NEW_MESSAGE
	};
	constexpr FdCanRxFifoParams FdCanRxFifo1 = {
			FDCAN_RX_FIFO1,
			FDCAN_FILTER_TO_RXFIFO1,
			FDCAN_IT_RX_FIFO1_NEW_MESSAGE,
			FDCAN_FLAG_RX_FIFO1_NEW_MESSAGE
	};


	class FdCanComm:public ICan{
		FDCAN_HandleTypeDef* const fdcan;

		std::unique_ptr<IRingBuffer<CanFrame> > rx_buff;
		std::unique_ptr<IRingBuffer<CanFrame> > tx_buff;

		const FdCanRxFifoParams &rx_fifo;

	public:
		FdCanComm(FDCAN_HandleTypeDef *_fdcan,std::unique_ptr<IRingBuffer<CanFrame>> _rx_buff,std::unique_ptr<IRingBuffer<CanFrame>> &&_tx_buff,const FdCanRxFifoParams &_rx_fifo)
			:fdcan(_fdcan),
		 	 rx_buff(std::move(_rx_buff)),
			 tx_buff(std::move(_tx_buff)),
			 rx_fifo(_rx_fifo){
		}

		void start(void){
			HAL_FDCAN_Start(fdcan);
			HAL_FDCAN_ActivateNotification(fdcan, rx_fifo.it, rx_fifo.it_buff);
			HAL_FDCAN_ActivateNotification(fdcan, FDCAN_IT_TX_COMPLETE, FDCAN_TX_BUFFER0 | FDCAN_TX_BUFFER1 | FDCAN_TX_BUFFER2);
		}

		////////////////
		//tx fucntions//
		////////////////
		uint32_t tx_available(void)const override{
			return tx_buff->get_free_level();
		}

		void tx_interrupt_task(void){
			while(HAL_FDCAN_GetTxFifoFreeLevel(fdcan) && tx_buff->get_busy_level()){
				std::optional<CanFrame> tx_frame = tx_buff->pop();
				if(not tx_frame.has_value()){
					break;
				}

				FDCAN_TxHeaderTypeDef tx_header;
				tx_header.Identifier = tx_frame.value().id;
				tx_header.IdType = tx_frame.value().is_ext_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
				tx_header.TxFrameType = tx_frame.value().is_remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
				tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				tx_header.BitRateSwitch = FDCAN_BRS_OFF;
				tx_header.FDFormat = FDCAN_CLASSIC_CAN;
				tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				tx_header.MessageMarker = 0;

#ifdef STM32G4xx_HAL_H
				tx_header.DataLength = tx_frame.value().data_length;
#elif STM32H7xx_HAL_H
				tx_header.DataLength = tx_frame.data_length<<16;
#endif
				HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &tx_header, const_cast<uint8_t*>(tx_frame.value().data));
			}
		}

		bool tx(const CanFrame &tx_frame)override{
			if(HAL_FDCAN_GetTxFifoFreeLevel(fdcan) > 0){
				FDCAN_TxHeaderTypeDef tx_header;
				tx_header.Identifier = tx_frame.id;
				tx_header.IdType = tx_frame.is_ext_id ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
				tx_header.TxFrameType = tx_frame.is_remote ? FDCAN_REMOTE_FRAME : FDCAN_DATA_FRAME;
				tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
				tx_header.BitRateSwitch = FDCAN_BRS_OFF;
				tx_header.FDFormat = FDCAN_CLASSIC_CAN;
				tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
				tx_header.MessageMarker = 0;

#ifdef STM32G4xx_HAL_H
				tx_header.DataLength = tx_frame.data_length;
#elif STM32H7xx_HAL_H
				tx_header.DataLength = tx_frame.data_length<<16;
#endif

				HAL_FDCAN_AddMessageToTxFifoQ(fdcan, &tx_header, const_cast<uint8_t*>(tx_frame.data));
			}else{
				if(!tx_buff->push(tx_frame)){
					return false;
				}
			}
			return true;
		}

		////////////////
		//rx functions//
		////////////////
		uint32_t rx_available(void)const override{
			return rx_buff->get_busy_level();
		}

		void rx_interrupt_task(void){
			FDCAN_RxHeaderTypeDef rx_header;
			CanFrame rx_frame;

			HAL_FDCAN_GetRxMessage(fdcan, rx_fifo.no, &rx_header, rx_frame.data);

			rx_frame.data_length = rx_header.DataLength;
			rx_frame.is_remote = rx_header.RxFrameType == FDCAN_REMOTE_FRAME ? true : false;
			rx_frame.is_ext_id = rx_header.IdType == FDCAN_EXTENDED_ID ? true : false;
			rx_frame.id = rx_header.Identifier;

#ifdef STM32G4xx_HAL_H
			rx_frame.data_length = rx_header.DataLength;
#elif STM32H7xx_HAL_H
			rx_frame.data_length = rx_header.DataLength>>16;
#endif

			rx_buff->push(rx_frame);
		}

		std::optional<CanFrame> rx(void)override{
			return rx_buff->pop();
		}

		bool set_filter_free(uint32_t filter_no,CanFilterMode fmode = CanFilterMode::ONLY_STD){
			assert(fmode != CanFilterMode::STD_AND_EXT);

			FDCAN_FilterTypeDef  filter;
			filter.IdType = fmode == CanFilterMode::ONLY_EXT ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
			filter.FilterIndex = filter_no;
			filter.FilterType = FDCAN_FILTER_MASK;
			filter.FilterConfig = rx_fifo.filter;
			filter.FilterID1 = 0x000;
			filter.FilterID2 = 0x000;

			if(HAL_FDCAN_ConfigFilter(fdcan, &filter)==HAL_OK){
				return true;
			}else{
				return false;
			}
		}
		bool set_filter(uint32_t filter_no,uint32_t id,uint32_t mask,CanFilterMode fmode){
			assert(fmode != CanFilterMode::STD_AND_EXT);

			FDCAN_FilterTypeDef  filter;
			filter.IdType = fmode == CanFilterMode::ONLY_EXT ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
			filter.FilterIndex = filter_no;
			filter.FilterType = FDCAN_FILTER_MASK;
			filter.FilterConfig = rx_fifo.filter;
			filter.FilterID1 = id;
			filter.FilterID2 = mask;

			if(HAL_FDCAN_ConfigFilter(fdcan, &filter)==HAL_OK){
				return true;
			}else{
				return false;
			}
		}

		FDCAN_HandleTypeDef *get_handler(void)const{
			return fdcan;
		}
	};
}

#endif //HAL_FDCAN_MODULE_ENABLED


#endif /* FDCAN_HPP_ */
