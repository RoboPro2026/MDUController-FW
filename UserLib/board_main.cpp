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
#include "CommonLib/slcan.hpp"
#include "CommonLib/serial_if.hpp"
#include "CommonLib/timer_interruption_control.hpp"
#include "CommonLib/usb_cdc.hpp"
#include "CommonLib/sequencer.hpp"

#include "LED_pattern.hpp"
#include "AMT21x_encoder.hpp"
#include "motor_control.hpp"
#include "vesc_data.hpp"
#include "motor_calibration.hpp"

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
			.set_abs_enc_uart(false,&huart5)
			.build();

	auto vesc = Blib::VescDataConverter{0};

	auto usb_cdc = Clib::UsbCdcComm{&hUsbDeviceFS,
		std::make_unique<Clib::RingBuffer<Clib::StrPack,4>>(),
		std::make_unique<Clib::RingBuffer<Clib::StrPack,4>>()
	};

}

namespace Test{
	auto sec_tim = Clib::InterruptionTimerHard{&htim17};

	float target = 1.0f;
	int16_t rm_val = 0;
	auto vrm = BoardLib::VirtualRobomasMotor(1000.0f,2,MReg::RobomasMD::C610,0.0004,0.001);
}

namespace be = BoardElement;

//メイン関数
extern "C"{
void cppmain(void){
	HAL_Delay(100);
	//be::can_main.set_filter_free(0,Clib::CanFilterMode::ONLY_EXT);
	be::can_main.set_filter(0,0x012,0x0FF,Clib::CanFilterMode::ONLY_STD);
	be::can_main.start();

	be::test_timer.set_task([](){
		Test::rm_val = Blib::RobomasMotorParam::torque_to_robomas_value(be::motor.get_motor_type(), be::motor.get_torque());
		auto cf = Test::vrm(Test::rm_val);
		be::motor.update(cf);
		be::md_state_led[2].update();
	});

	Test::sec_tim.set_task([](){
		Test::target *= -1.0f;
	});
	be::test_timer.start_timer(0.001f);
	Test::sec_tim.start_timer(1.0f);

	be::motor.start_calibration();
	while(be::motor.is_calibrating()){
		be::md_state_led[2].play(Blib::LEDPattern::test,false);
		HAL_Delay(10);
	}
	printf("%f,%f\r\n",be::motor.dob.inverse_model.get_inertia(),be::motor.dob.inverse_model.get_friction_coef());

	be::motor.overwrite_rad(0.0f);
	be::motor.set_control_mode(MReg::ControlMode::POSITION);
	be::motor.use_dob(true);
	while(1){
		be::md_state_led[2].play(Blib::LEDPattern::abs_speed_mode,false);
		be::motor.set_target_rad(Test::target);

		printf("%4.3f,%4.3f,%4.3f\r\n",be::motor.get_overwrited_rad(),be::motor.enc.get_rad_speed(),be::motor.enc.get_torque());

		HAL_Delay(1);
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//割り込み関数たち
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


//uart(rs485
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

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



