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
		static constexpr size_t enc_resolution = 12;
		UART_HandleTypeDef* const uart;

		const uint8_t enc_id = 0x36;

		int32_t coef_inv;

		uint8_t enc_val[2] = {0};

		enum class State{
			REQUESTING,
			WAITING_RESPOSNCE,
			DATA_READY,
		};
		State state;

	public:
		AMT21xEnc(UART_HandleTypeDef* _uart,uint8_t _enc_id,float update_freq,bool is_inv = false)
			:ContinuableEncoder(enc_resolution,update_freq),
			 uart(_uart),
			 enc_id(_enc_id),
			 coef_inv(is_inv?-1:1){
			state = State::REQUESTING;
		}

		bool is_ready(void){
			return (state == State::DATA_READY) ? true : false;
		}

		bool is_inv(void){
			return (coef_inv == 1) ? false : true;
		}
		void is_inv(bool is_inv){
			coef_inv = is_inv ? 1 : -1;
		}


		//エンコーダとの通信に使用する関数群
		//request_positon
		//↓
		//tx_interupt_task(送信完了割り込み内）
		//↓
		//rx_interrupt_task(受信完了割り込み内）の順に実行すること
		void request_position(void){
			HAL_UART_Transmit_IT(uart, const_cast<uint8_t*>(&enc_id),1);
			state = State::REQUESTING;
		}
		//HAL_UART_TxCpltCallback内におくこと
		void tx_interrupt_task(void){
			HAL_UART_Receive_IT(uart, enc_val, 2);
			state = State::WAITING_RESPOSNCE;
		}

		//HAL_UART_RxCpltCallback内におくこと
		void rx_interrupt_task(void){
			uint16_t raw_angle = enc_val[1]<<8 | enc_val[0];
			update(raw_angle*coef_inv);
			state = State::DATA_READY;
		}

		UART_HandleTypeDef* get_handler(void){
			return uart;
		}
	};
}

#endif //HAL_UART_MODULE_ENABLED

#endif /* AMT21X_ENCODER_HPP_ */
