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
#include "vesc_data.hpp"

#include <array>
#include <bit>
#include <stdio.h>

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

	auto LED_r = Clib::Sequencer<Clib::PWMHard>{Clib::PWMHard{&htim1,TIM_CHANNEL_2}};
	auto LED_g = Clib::Sequencer<Clib::PWMHard>{Clib::PWMHard{&htim1,TIM_CHANNEL_3}};
	auto LED_b = Clib::Sequencer<Clib::PWMHard>{Clib::PWMHard{&htim1,TIM_CHANNEL_4}};

	auto md_state_led = std::array<Clib::Sequencer<Clib::GPIO>,3>{
		Clib::GPIO{LED0_GPIO_Port,LED0_Pin},
		Clib::GPIO{LED1_GPIO_Port,LED1_Pin},
		Clib::GPIO{LED2_GPIO_Port,LED2_Pin},
		//Clib::GPIO{LED3_GPIO_Port,LED3_Pin}
	};

	auto encs = std::array<Blib::AMT21xEnc,4>{
		Blib::AMT21xEnc(&huart5,0x54,1000.0f),
		Blib::AMT21xEnc(&huart3,0x54,1000.0f),
		Blib::AMT21xEnc(&hlpuart1,0x54,1000.0f),
		Blib::AMT21xEnc(&huart2,0x54,1000.0f),
	};


	auto test_timer = Clib::InterruptionTimerHard{&htim15};

	auto motor = Blib::C6x0ControllerBuilder(0,MReg::RobomasMD::C610).build();

	auto dob_test = Clib::Math::DisturbanceObserver<Blib::MotorInverceModel>{
		1000.0f,
		Blib::MotorInverceModel(1000.0f,1.0f,1.0f),
		5.0
	};

	auto vesc = Blib::VescDataConverter{0};

	auto usb_cdc = Clib::UsbCdcComm{&hUsbDeviceFS,
		std::make_unique<Clib::RingBuffer<Clib::StrPack,4>>(),
		std::make_unique<Clib::RingBuffer<Clib::StrPack,4>>()
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
	}
}

//auto filter = Clib::Math::LowpassFilterBD<float>(5000.0f,100.0f);
auto filter = Clib::Math::BiquadFilter<float>(5000.0f,200.0f,50.0f);
//auto filter = Clib::Math::HighpassFilterBD<float>(5000.0f,500.0f);

//メイン関数
extern "C"{
void cppmain(void){
	printf("start\r\n");
	HAL_Delay(100);
	//be::can_main.set_filter_free(0,Clib::CanFilterMode::ONLY_EXT);
	be::can_main.set_filter(0,0x012,0x0FF,Clib::CanFilterMode::ONLY_STD);
	be::can_main.start();

	be::test_timer.set_task([](){
		static int cnt = 0;
		Clib::CanFrame cf_dummy;
		cf_dummy.id = 0x201;
		cf_dummy.data_length = 8;
		cf_dummy.data[0] = ++cnt;
		cf_dummy.data[3] = ++cnt;

		be::motor.update(cf_dummy);

		be::md_state_led[2].update();
	});
	be::test_timer.start_timer(0.001f);


	be::motor.set_control_mode(MReg::ControlMode::POSITION);
	be::motor.use_dob(true);
	printf("loop start\r\n");

	while(1){
		be::md_state_led[2].play(Blib::LEDPattern::abs_speed_mode,false);
		HAL_Delay(100);

		//can test
		Clib::Protocol::DataPacket dp;
		Clib::CanFrame cf;
		Clib::StrPack sd;

		dp.board_ID = 2;
		dp.priority = 1;
		dp.data_type = Clib::Protocol::DataType::COMMON_ID;
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
			auto rx_cf = be::can_main.rx();
			printf("rx can!\r\n");
		}
		sd.size = Clib::SLCAN::can_to_slcan(cf,(char*)(sd.data),sd.max_size);

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



