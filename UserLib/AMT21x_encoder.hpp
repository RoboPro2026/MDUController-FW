/*
 * AMT212.hpp
 *
 *  Created on: Jul 10, 2025
 *      Author: gomas
 */

#ifndef AMT21X_ENCODER_HPP_
#define AMT21X_ENCODER_HPP_

#include "main.h"

#ifdef HAL_UART_MODULE_ENABLED

#include "CommonLib/encoder.hpp"

namespace BoardLib{
	class AMT21xEnc:public CommonLib::ContinuableEncoder{
	private:
		static constexpr size_t enc_resolution = 14;
		UART_HandleTypeDef* const uart;

		const uint8_t enc_id = 0x36;

		int32_t coef_inv;

		uint8_t enc_val[2] = {0};

		bool new_data_available;
		bool no_responce;

	public:
		AMT21xEnc(UART_HandleTypeDef* _uart,uint8_t _enc_id = 0x54,float update_freq = 1000.0f,bool is_inv = false)
			:ContinuableEncoder(enc_resolution,update_freq),
			 uart(_uart),
			 enc_id(_enc_id),
			 coef_inv(is_inv?-1:1),
			 new_data_available(false),
			 no_responce(false){
		}

		bool is_ready(void){
			return new_data_available;
		}

		bool is_inv(void){
			return (coef_inv == 1) ? false : true;
		}
		void is_inv(bool is_inv){
			coef_inv = is_inv ? 1 : -1;
		}

		bool is_dead(void)const{
			return no_responce;
		}


		//エンコーダとの通信に使用する関数群
		//request_positon
		//↓
		//rx_interrupt_task(受信完了割り込み内）
		void request_position(void){
			HAL_UART_Transmit_IT(uart, const_cast<uint8_t*>(&enc_id),1);
			HAL_UART_Receive_IT(uart, enc_val, 2);

			if(not new_data_available){
				no_responce = true;
				return;
			}
			new_data_available = false;
		}

		//HAL_UART_RxCpltCallback内におくこと
		void rx_interrupt_task(void){
			uint16_t raw_angle = enc_val[1]<<8 | enc_val[0];
			update(raw_angle*coef_inv);
			no_responce = false;
			new_data_available = true;
		}

		UART_HandleTypeDef* get_handler(void){
			return uart;
		}
	};
}

#endif //HAL_UART_MODULE_ENABLED

#endif /* AMT21X_ENCODER_HPP_ */
