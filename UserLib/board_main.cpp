/*
 * board_main.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: gomas
 */
#include "main.h"

#include "CommonLib/fdcan_control.hpp"
#include "CommonLib/gpio.hpp"
#include "CommonLib/sequencable_io.hpp"
#include "CommonLib/slcan.hpp"
#include "CommonLib/serial_if.hpp"
#include "AMT212.hpp"

#include <array>
#include <stdio.h>

extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern TIM_HandleTypeDef htim1;


namespace BoardElement{
	namespace TmpMemoryPool{
		uint8_t can_main_tx_buff[sizeof(SabaneLib::RingBuffer<SabaneLib::CanFrame,5>)];
		uint8_t can_main_rx_buff[sizeof(SabaneLib::RingBuffer<SabaneLib::CanFrame,5>)];
		uint8_t can_md_tx_buff[sizeof(SabaneLib::RingBuffer<SabaneLib::CanFrame,5>)];
		uint8_t can_md_rx_buff[sizeof(SabaneLib::RingBuffer<SabaneLib::CanFrame,5>)];

		uint8_t led_r_pwm[sizeof(SabaneLib::PWMHard)];
		uint8_t led_g_pwm[sizeof(SabaneLib::PWMHard)];
		uint8_t led_b_pwm[sizeof(SabaneLib::PWMHard)];

		uint8_t led0_gpio[sizeof(SabaneLib::GPIO)];
		uint8_t led1_gpio[sizeof(SabaneLib::GPIO)];
		uint8_t led2_gpio[sizeof(SabaneLib::GPIO)];
		uint8_t led3_gpio[sizeof(SabaneLib::GPIO)];
	}

	auto encs = std::array<SabaneLib::AMT21xState,4>{
		SabaneLib::AMT21xState(&huart5,0x54,1000.0f),
		SabaneLib::AMT21xState(&huart3,0x54,1000.0f),
		SabaneLib::AMT21xState(&hlpuart1,0x54,1000.0f),
		SabaneLib::AMT21xState(&huart2,0x54,1000.0f),
	};

	auto can_main = SabaneLib::FdCanComm{
		&hfdcan2,
		std::unique_ptr<SabaneLib::RingBuffer<SabaneLib::CanFrame,5>>(
				new(TmpMemoryPool::can_main_tx_buff) SabaneLib::RingBuffer<SabaneLib::CanFrame,5>{}),
		std::unique_ptr<SabaneLib::RingBuffer<SabaneLib::CanFrame,5>>(
				new(TmpMemoryPool::can_main_rx_buff) SabaneLib::RingBuffer<SabaneLib::CanFrame,5>{}),
		SabaneLib::FdCanRxFifo0
	};

	auto can_md = SabaneLib::FdCanComm{
		&hfdcan3,
		std::unique_ptr<SabaneLib::RingBuffer<SabaneLib::CanFrame,5>>(
				new(TmpMemoryPool::can_md_tx_buff) SabaneLib::RingBuffer<SabaneLib::CanFrame,5>{}),
		std::unique_ptr<SabaneLib::RingBuffer<SabaneLib::CanFrame,5>>(
				new(TmpMemoryPool::can_md_rx_buff) SabaneLib::RingBuffer<SabaneLib::CanFrame,5>{}),
		SabaneLib::FdCanRxFifo0
	};

	auto LED_r = SabaneLib::SequencableIO<SabaneLib::PWMHard>{
			std::unique_ptr<SabaneLib::PWMHard> (new(TmpMemoryPool::led_r_pwm) SabaneLib::PWMHard{&htim1,TIM_CHANNEL_2})
	};
	auto LED_g = SabaneLib::SequencableIO<SabaneLib::PWMHard>{
			std::unique_ptr<SabaneLib::PWMHard> (new(TmpMemoryPool::led_g_pwm) SabaneLib::PWMHard{&htim1,TIM_CHANNEL_3})
	};
	auto LED_b = SabaneLib::SequencableIO<SabaneLib::PWMHard>{
			std::unique_ptr<SabaneLib::PWMHard> (new(TmpMemoryPool::led_b_pwm) SabaneLib::PWMHard{&htim1,TIM_CHANNEL_4})
	};

	auto md_state_led = std::array<SabaneLib::SequencableIO<SabaneLib::GPIO>,4>{
		std::unique_ptr<SabaneLib::GPIO> (new(TmpMemoryPool::led0_gpio) SabaneLib::GPIO{LED0_GPIO_Port,LED1_Pin}),
		std::unique_ptr<SabaneLib::GPIO> (new(TmpMemoryPool::led1_gpio) SabaneLib::GPIO{LED1_GPIO_Port,LED1_Pin}),
		std::unique_ptr<SabaneLib::GPIO> (new(TmpMemoryPool::led2_gpio) SabaneLib::GPIO{LED2_GPIO_Port,LED2_Pin}),
		std::unique_ptr<SabaneLib::GPIO> (new(TmpMemoryPool::led3_gpio) SabaneLib::GPIO{LED3_GPIO_Port,LED3_Pin})
	};
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//割り込み関数たち
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace be = BoardElement;

//uart(rs485
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == be::encs[0].get_handler()){
		be::encs[0].tx_interrupt_task();
	}else if(huart == be::encs[1].get_handler()){
		be::encs[1].tx_interrupt_task();
	}else if(huart == be::encs[2].get_handler()){
		be::encs[2].tx_interrupt_task();
	}else if(huart == be::encs[3].get_handler()){
		be::encs[3].tx_interrupt_task();
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == be::encs[0].get_handler()){
		be::encs[0].rx_interrupt_task();
	}else if(huart == be::encs[1].get_handler()){
		be::encs[1].rx_interrupt_task();
	}else if(huart == be::encs[2].get_handler()){
		be::encs[2].rx_interrupt_task();
	}else if(huart == be::encs[3].get_handler()){
		be::encs[3].rx_interrupt_task();
	}
}

//can
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if(hfdcan == be::can_main.get_handler()){
		be::can_main.rx_interrupt_task();
	}else if(hfdcan == be::can_md.get_handler()){
		be::can_md.rx_interrupt_task();
	}
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes){
	if(hfdcan == be::can_main.get_handler()){
		be::can_main.tx_interrupt_task();
	}else if(hfdcan == be::can_md.get_handler()){
		be::can_md.tx_interrupt_task();
	}
	be::md_state_led[2].out_weak(1.0);
}

//timer


//メイン関数
extern "C"{
void cppmain(void){
	HAL_Delay(500);
	printf("start\r\n");
	HAL_Delay(100);
	be::can_main.set_filter_free(0,SabaneLib::CanFilterMode::ONLY_EXT);
	be::can_main.start();
	printf("can init\r\n");
//	//TODO:STD_AND_EXTが本当に使えないのか実験
//	be::can_md.set_filter_free(0,SabaneLib::CanFilterMode::ONLY_STD);
//	be::can_md.start();
//
//	be::LED_r.io->start();
//	be::LED_g.io->start();
//	be::LED_b.io->start();

	be::md_state_led[2].out_weak(1.0);

	while(1){
		be::md_state_led[2].out_weak(1.0);
		HAL_Delay(100);
		be::md_state_led[2].out_weak(0.0);
		HAL_Delay(100);

		SabaneLib::Protocol::DataPacket dp;
		SabaneLib::CanFrame cf;
		SabaneLib::SerialData sd;

		dp.board_ID = 2;
		dp.priority = 1;
		dp.data_type = SabaneLib::Protocol::DataType::RMC_DATA;
		dp.writer().write<int32_t>(0x0123'4567);
		cf.decode_common_data_packet(dp);

		printf("can buff:%d\r\n",be::can_main.tx_available());

		cf.is_ext_id = true;
		be::can_main.tx(cf);
		printf("can tx\r\n");

		HAL_Delay(100);
		if(be::can_main.rx_available()){
			be::can_main.rx(cf);
			printf("rx can!\r\n");
		}
		sd.size = SabaneLib::SLCAN::can_to_slcan(cf,(char*)(sd.data),sd.max_size);

		printf("hello:%s\r\n",sd.data);
//		for(auto &e: BoardElement::encs){
//			e.request_position();
//		}
//		HAL_Delay(100);
	}
}

int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len,100);
	return len;
}

}



