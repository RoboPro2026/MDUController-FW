/*
 * board_main.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: gomas
 */
#include "main.h"

#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "CommonLib/fdcan_control.hpp"
#include "CommonLib/gpio.hpp"
#include "CommonLib/slcan.hpp"
#include "CommonLib/serial_if.hpp"
#include "CommonLib/timer_interruption_control.hpp"
#include "CommonLib/usb_cdc.hpp"
#include "CommonLib/sequencer.hpp"
#include "flash_management.hpp"

#include "LED_pattern.hpp"
#include "motor_control.hpp"
#include "vesc_data.hpp"
#include "motor_calibration.hpp"

#include <array>
#include <bit>
#include <stdio.h>
#include "abs_encoder.hpp"


extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim15;
extern TIM_HandleTypeDef htim16;
extern TIM_HandleTypeDef htim17;

extern USBD_HandleTypeDef hUsbDeviceFS;

namespace Clib = CommonLib;
namespace Blib = BoardLib;

namespace BoardElement{
	namespace TmpMemoryPool{
		uint8_t can_main_tx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,5>)];
		uint8_t can_main_rx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,5>)];
		uint8_t can_md_tx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,5>)];
		uint8_t can_md_rx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,5>)];

		uint8_t abs_enc0[sizeof(BoardLib::AMT21xEnc)];
		uint8_t abs_enc1[sizeof(BoardLib::AMT21xEnc)];
		uint8_t abs_enc2[sizeof(BoardLib::AMT21xEnc)];
		uint8_t abs_enc3[sizeof(BoardLib::AMT21xEnc)];
	}

	auto can_main = Clib::FdCanComm{
		&hfdcan2,
		std::unique_ptr<Clib::RingBuffer<Clib::CanFrame,5>>(
				new(TmpMemoryPool::can_main_tx_buff) Clib::RingBuffer<Clib::CanFrame,5>{}),
		std::unique_ptr<Clib::RingBuffer<Clib::CanFrame,5>>(
				new(TmpMemoryPool::can_main_rx_buff) Clib::RingBuffer<Clib::CanFrame,5>{}),
		Clib::FdCanRxFifo0
	};

	auto can_md = Clib::FdCanComm{
		&hfdcan3,
		std::unique_ptr<Clib::RingBuffer<Clib::CanFrame,5>>(
				new(TmpMemoryPool::can_md_tx_buff) Clib::RingBuffer<Clib::CanFrame,5>{}),
		std::unique_ptr<Clib::RingBuffer<Clib::CanFrame,5>>(
				new(TmpMemoryPool::can_md_rx_buff) Clib::RingBuffer<Clib::CanFrame,5>{}),
		Clib::FdCanRxFifo1
	};

	auto led_r = Clib::PWMHard{&htim1,TIM_CHANNEL_2};
	auto led_g = Clib::PWMHard{&htim1,TIM_CHANNEL_3};
	auto led_b = Clib::PWMHard{&htim1,TIM_CHANNEL_4};
	auto led0 = Clib::GPIO{LED0_GPIO_Port,LED0_Pin};
	auto led1 = Clib::GPIO{LED1_GPIO_Port,LED1_Pin};
	auto led2 = Clib::GPIO{LED2_GPIO_Port,LED2_Pin};
	auto led3 = Clib::GPIO{LED3_GPIO_Port,LED3_Pin};

	auto led_r_seqencer = Clib::Sequencer{[](float v){led_r(v);}};
	auto led_g_seqencer = Clib::Sequencer{[](float v){led_g(v);}};
	auto led_b_seqencer = Clib::Sequencer{[](float v){led_b(v);}};

	auto md_state_led = std::array<Clib::Sequencer,4>{
		Clib::Sequencer([](float v){led0(v>0.0f);}),
		Clib::Sequencer([](float v){led1(v>0.0f);}),
		Clib::Sequencer([](float v){led2(v>0.0f);}),
		Clib::Sequencer([](float v){led3(v>0.0f);})
	};

	auto test_timer = Clib::InterruptionTimerHard{&htim15};

	auto motor = Blib::C6x0ControllerBuilder(2,MReg::RobomasMD::C610)
			.set_abs_enc(std::unique_ptr<Blib::IABSEncoder>(new(TmpMemoryPool::abs_enc) Blib::AMT21xEnc(&huart5)),false)
			.build();


	auto vesc = Blib::VescDataConverter{0};

	auto usb_cdc = Clib::UsbCdcComm{&hUsbDeviceFS,
		std::make_unique<Clib::RingBuffer<Clib::StrPack,4>>(),
		std::make_unique<Clib::RingBuffer<Clib::StrPack,4>>()
	};

}

namespace be = BoardElement;

//メイン関数
extern "C"{
void cppmain(void){
	init();
	while(1){

	}
}

int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len,100);
	return len;
}

void usb_cdc_rx_callback(const uint8_t *input,size_t size){
	be::usb_cdc.rx_interrupt_task(input, size);
}

}//extern "C"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//割り込み関数たち
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//uart(rs485
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){

}

//can
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	if(hfdcan == be::can_main.get_handler()){
		be::can_main.rx_interrupt_task();
	}else if(hfdcan == be::can_md.get_handler()){
		be::can_md.rx_interrupt_task();
	}
	//be::md_state_led[2].play(Blib::LEDPattern::ok,false);
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if(hfdcan == be::can_main.get_handler()){
		be::can_main.rx_interrupt_task();
	}else if(hfdcan == be::can_md.get_handler()){
		be::can_md.rx_interrupt_task();
	}
	//be::md_state_led[2].play(Blib::LEDPattern::ok,false);
}

void HAL_FDCAN_TxBufferCompleteCallback(FDCAN_HandleTypeDef *hfdcan, uint32_t BufferIndexes){
	if(hfdcan == be::can_main.get_handler()){
		be::can_main.tx_interrupt_task();
	}else if(hfdcan == be::can_md.get_handler()){
		be::can_md.tx_interrupt_task();
	}
}

//timer
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim == be::test_timer.get_handler()){
		be::test_timer.interrupt_task();
	}else if(htim == Test::sec_tim.get_handler()){
		Test::sec_tim.interrupt_task();
	}
}



