/*
 * board_main.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: gomas
 */
#include "main.h"

#include "usbd_cdc_if.h"
#include "usb_device.h"

#include "CommonLib/Math/filter.hpp"
#include "CommonLib/Math/disturbance_observer.hpp"

#include "CommonLib/fdcan_control.hpp"
#include "CommonLib/gpio.hpp"
#include "CommonLib/sequencable_io.hpp"
#include "CommonLib/slcan.hpp"
#include "CommonLib/serial_if.hpp"
#include "CommonLib/timer_interruption_control.hpp"

#include "CommonLib/usb_cdc.hpp"

#include "LED_pattern.hpp"
#include "AMT21x_encoder.hpp"
#include "motor_control.hpp"

#include <array>
#include <stdio.h>
#include <bit>

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

namespace BoardElement{
	namespace TmpMemoryPool{
		uint8_t can_main_tx_buff[sizeof(CommonLib::RingBuffer<CommonLib::CanFrame,5>)];
		uint8_t can_main_rx_buff[sizeof(CommonLib::RingBuffer<CommonLib::CanFrame,5>)];
		uint8_t can_md_tx_buff[sizeof(CommonLib::RingBuffer<CommonLib::CanFrame,5>)];
		uint8_t can_md_rx_buff[sizeof(CommonLib::RingBuffer<CommonLib::CanFrame,5>)];
	}

	auto can_main = CommonLib::FdCanComm{
		&hfdcan2,
		std::unique_ptr<CommonLib::RingBuffer<CommonLib::CanFrame,5>>(
				new(TmpMemoryPool::can_main_tx_buff) CommonLib::RingBuffer<CommonLib::CanFrame,5>{}),
		std::unique_ptr<CommonLib::RingBuffer<CommonLib::CanFrame,5>>(
				new(TmpMemoryPool::can_main_rx_buff) CommonLib::RingBuffer<CommonLib::CanFrame,5>{}),
		CommonLib::FdCanRxFifo0
	};

	auto can_md = CommonLib::FdCanComm{
		&hfdcan3,
		std::unique_ptr<CommonLib::RingBuffer<CommonLib::CanFrame,5>>(
				new(TmpMemoryPool::can_md_tx_buff) CommonLib::RingBuffer<CommonLib::CanFrame,5>{}),
		std::unique_ptr<CommonLib::RingBuffer<CommonLib::CanFrame,5>>(
				new(TmpMemoryPool::can_md_rx_buff) CommonLib::RingBuffer<CommonLib::CanFrame,5>{}),
		CommonLib::FdCanRxFifo1
	};

	auto LED_r = CommonLib::SequencableIO<CommonLib::PWMHard>{CommonLib::PWMHard{&htim1,TIM_CHANNEL_2}};
	auto LED_g = CommonLib::SequencableIO<CommonLib::PWMHard>{CommonLib::PWMHard{&htim1,TIM_CHANNEL_3}};
	auto LED_b = CommonLib::SequencableIO<CommonLib::PWMHard>{CommonLib::PWMHard{&htim1,TIM_CHANNEL_4}};

	auto md_state_led = std::array<CommonLib::SequencableIO<CommonLib::GPIO>,3>{
		CommonLib::GPIO{LED0_GPIO_Port,LED0_Pin},
		CommonLib::GPIO{LED1_GPIO_Port,LED1_Pin},
		CommonLib::GPIO{LED2_GPIO_Port,LED2_Pin},
		//CommonLib::GPIO{LED3_GPIO_Port,LED3_Pin}
	};

	auto encs = std::array<BoardLib::AMT21xEnc,4>{
		BoardLib::AMT21xEnc(&huart5,0x54,1000.0f),
		BoardLib::AMT21xEnc(&huart3,0x54,1000.0f),
		BoardLib::AMT21xEnc(&hlpuart1,0x54,1000.0f),
		BoardLib::AMT21xEnc(&huart2,0x54,1000.0f),
	};


	auto test_timer = CommonLib::InterruptionTimerHard{&htim15};

	auto motor = BoardLib::C6x0Controller{1};

	auto dob_test = CommonLib::Math::DisturbanceObserver<BoardLib::MotorInverceModel>{
		1000.0f,
		BoardLib::MotorInverceModel(1000.0f,1.0f,1.0f),
		5.0
	};

	auto usb_cdc = CommonLib::UsbCdcComm{&hUsbDeviceFS,
		std::make_unique<CommonLib::RingBuffer<CommonLib::SerialData,4>>(),
		std::make_unique<CommonLib::RingBuffer<CommonLib::SerialData,4>>()
	};

}

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//割り込み関数たち
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
namespace be = BoardElement;

//uart(rs485
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
	//be::md_state_led[2].play(BoardLib::LEDPattern::ok,false);
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	if(hfdcan == be::can_main.get_handler()){
		be::can_main.rx_interrupt_task();
	}else if(hfdcan == be::can_md.get_handler()){
		be::can_md.rx_interrupt_task();
	}
	//be::md_state_led[2].play(BoardLib::LEDPattern::ok,false);
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
	}
}

//auto filter = CommonLib::Math::LowpassFilterBD<float>(5000.0f,100.0f);
auto filter = CommonLib::Math::BiquadFilter<float>(5000.0f,200.0f,50.0f);
//auto filter = CommonLib::Math::HighpassFilterBD<float>(5000.0f,500.0f);

//メイン関数
extern "C"{
void cppmain(void){

	HAL_Delay(500);
	printf("start\r\n");
	HAL_Delay(100);
	//be::can_main.set_filter_free(0,CommonLib::CanFilterMode::ONLY_EXT);
	be::can_main.set_filter(0,0x012,0x0FF,CommonLib::CanFilterMode::ONLY_STD);
	be::can_main.start();
	printf("can init\r\n");

	printf("tim15_f:%d\r\n",CommonLib::TimerHelper::get_timer_clock_freq(be::test_timer.get_handler()->Instance));

	be::test_timer.set_task([](){
		be::md_state_led[2].update();
	});
	be::test_timer.start_timer(0.001f);
	printf("tim15_period:%f\r\n",be::test_timer.get_timer_period());
	printf("tim1 clock:%d\r\n",CommonLib::TimerHelper::get_timer_clock_freq(htim1.Instance));
	printf("tim15 clock:%d\r\n",CommonLib::TimerHelper::get_timer_clock_freq(htim15.Instance));
	printf("tim16 clock:%d\r\n",CommonLib::TimerHelper::get_timer_clock_freq(htim16.Instance));
	printf("tim17 clock:%d\r\n",CommonLib::TimerHelper::get_timer_clock_freq(htim17.Instance));

//	be::LED_r.io->start();
//	be::LED_g.io->start();
//	be::LED_b.io->start();

	while(1){
		be::md_state_led[2].play(BoardLib::LEDPattern::test,false);

//		//can test
		CommonLib::Protocol::DataPacket dp;
		CommonLib::CanFrame cf;
		CommonLib::SerialData sd;

		dp.board_ID = 2;
		dp.priority = 1;
		dp.data_type = CommonLib::Protocol::DataType::RMC_DATA;
		dp.writer().write<int32_t>(0x0123'4567);
		cf.decode_common_data_packet(dp);

		printf("can buff:%d\r\n",be::can_main.tx_available());

		cf.is_ext_id = false;
		cf.id = 0x012;
		cf.data_length = 8;
		be::can_main.tx(cf);
		printf("can tx\r\n");

		HAL_Delay(100);
		if(be::can_main.rx_available()){
			be::can_main.rx(cf);
			printf("rx can!\r\n");
		}
		sd.size = CommonLib::SLCAN::can_to_slcan(cf,(char*)(sd.data),sd.max_size);

		printf("hello:%s\r\n",sd.data);

		for(auto &e: BoardElement::encs){
			e.request_position();
		}
		HAL_Delay(100);
	}
}

int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len,100);
	return len;
}

void usb_cdc_rx_callback(const uint8_t *input,size_t size){
	be::usb_cdc.rx_interrupt_task(input, size);
}

}



