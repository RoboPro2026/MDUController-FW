/*
 * serial_if.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_SERIAL_IF_HPP_
#define COMMONLIB_SERIAL_IF_HPP_

#include "main.h"
#include "ring_buffer.hpp"
#include <memory>

namespace CommonLib{

struct SerialData{
	static constexpr size_t max_size = 64;
	uint8_t data[max_size] = {0};
	size_t size = 0;
};

class ISerial{
public:
	virtual bool tx(const SerialData &data) = 0;
	virtual size_t tx_available(void) const = 0;

	virtual bool rx(SerialData &data) = 0;
	virtual size_t rx_available(void) const = 0;

	virtual ~ISerial(){}
};



#ifdef HAL_UART_MODULE_ENABLED
///////////////////////////////////////////////////////////////////////////////
//uart通信
///////////////////////////////////////////////////////////////////////////////
class UartComm : ISerial{
private:
	UART_HandleTypeDef* uart;
	std::unique_ptr<IRingBuffer<SerialData>> rx_buff;

	uint8_t rx_tmp_byte;
	SerialData rx_tmp_packet;

	SerialData tx_tmp_packet;
	bool is_transmitting = false;
public:
	UartComm(UART_HandleTypeDef *_uart,std::unique_ptr<IRingBuffer<SerialData>> _rx_buff):
		uart(_uart),
		rx_buff(std::move(_rx_buff)){
	}

	UART_HandleTypeDef *get_handle(void)const{
		return uart;
	}

	//tx functions
	bool tx(const SerialData &data) override{
		if(is_transmitting){
			return false;
		}else{
			tx_tmp_packet = data;
			HAL_UART_Transmit_IT(uart, const_cast<uint8_t*>(tx_tmp_packet.data), tx_tmp_packet.size);
			is_transmitting = true;
			return true;
		}
	}
	size_t tx_available(void)const override{
		return is_transmitting ? 0:1;
	}
	void tx_interrupt_task(void){ //送信完了割り込みで呼ぶこと
		is_transmitting = false;
	}

	//rx functions
	void rx_start(void){
		HAL_UART_Receive_IT(uart, &rx_tmp_byte, 1);
	}
	bool rx(SerialData &data) override{
		return rx_buff->pop(data);
	}
	size_t rx_available(void) const override{
		return rx_buff->get_busy_level();
	}

	void rx_interrupt_task(void){ //受信完了割り込みで呼ぶこと
		if((rx_tmp_byte=='\r') || (rx_tmp_byte=='\n') || (rx_tmp_byte=='\0') || (rx_tmp_packet.size >= rx_tmp_packet.max_size-1)){
			rx_tmp_packet.data[rx_tmp_packet.size] = rx_tmp_byte;
			rx_tmp_packet.size ++;

			rx_buff->push(rx_tmp_packet);

			rx_tmp_packet = SerialData{};
		}else{
			rx_tmp_packet.data[rx_tmp_packet.size] = rx_tmp_byte;
			rx_tmp_packet.size ++;
		}

		rx_start();
	}
};

#endif //HAL_UART_MODULE_ENABLED
}


#endif /* COMMONLIB_SERIAL_IF_HPP_ */
