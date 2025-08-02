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
#include "control_unit.hpp"

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
extern TIM_HandleTypeDef htim2;

extern USBD_HandleTypeDef hUsbDeviceFS;

namespace Clib = CommonLib;
namespace Blib = BoardLib;

namespace BoardElement{
	namespace TmpMemoryPool{
		uint8_t can_main_tx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,5>)];
		uint8_t can_main_rx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,5>)];
		uint8_t can_md_tx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,5>)];
		uint8_t can_md_rx_buff[sizeof(Clib::RingBuffer<Clib::CanFrame,5>)];
		uint8_t usb_rx_buff[sizeof(Clib::RingBuffer<Clib::StrPack,5>)];
		uint8_t usb_tx_buff[sizeof(Clib::RingBuffer<Clib::StrPack,5>)];

		uint8_t abs_enc0[sizeof(BoardLib::AMT21xEnc)];
		uint8_t abs_enc1[sizeof(BoardLib::AMT21xEnc)];
		uint8_t abs_enc2[sizeof(BoardLib::AMT21xEnc)];
		uint8_t abs_enc3[sizeof(BoardLib::AMT21xEnc)];

		uint8_t tim_monitor[sizeof(Clib::InterruptionTimerHard)];
		uint8_t tim_can_timeout[sizeof(Clib::InterruptionTimerHard)];
	}

	constexpr size_t MOTOR_N = 4;

	constexpr auto default_init_param = Blib::MotorControlParam{
		.mode = 0,
		.trq_limit = 1.0f,
		.spd_gain_p = 0.5f,
		.spd_gain_i = 0.1f,
		.spd_gain_d = 0.0f,

		.spd_limit = 314.0f,
		.pos_gain_p = 5.0f,
		.pos_gain_i = 0.0f,
		.pos_gain_d = 0.0f,

		.dob_j = 0.0004f,
		.dob_d = 0.0005f
	};

	struct MotorInitParam{
		int never_writed;
		std::array<Blib::MotorControlParam,MOTOR_N> param;
	};

	auto init_params = MotorInitParam();

	size_t board_id = 0x0;

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

	auto usb_cdc = Clib::UsbCdcComm{&hUsbDeviceFS,
		std::unique_ptr<Clib::RingBuffer<Clib::StrPack,5>>(
				new(TmpMemoryPool::usb_rx_buff) Clib::RingBuffer<Clib::StrPack,5>{}),
		std::unique_ptr<Clib::RingBuffer<Clib::StrPack,5>>(
				new(TmpMemoryPool::usb_tx_buff) Clib::RingBuffer<Clib::StrPack,5>{}),
	};

	auto led_r = Clib::PWMHard{&htim1,TIM_CHANNEL_2};
	auto led_g = Clib::PWMHard{&htim1,TIM_CHANNEL_3};
	auto led_b = Clib::PWMHard{&htim1,TIM_CHANNEL_4};
	auto led_r_sequencer = Clib::Sequencer{[](float v){led_r(1.0-v);}};
	auto led_g_sequencer = Clib::Sequencer{[](float v){led_g(1.0-v);}};
	auto led_b_sequencer = Clib::Sequencer{[](float v){led_b(1.0-v);}};

	auto id_pins = std::array<Clib::GPIO,MOTOR_N>{
		Clib::GPIO{ID0_GPIO_Port,ID0_Pin},
		Clib::GPIO{ID1_GPIO_Port,ID1_Pin},
		Clib::GPIO{ID2_GPIO_Port,ID2_Pin},
		Clib::GPIO{ID3_GPIO_Port,ID3_Pin}
	};

	auto tim_1khz  = Clib::InterruptionTimerHard{&htim15};
	auto tim_100hz = Clib::InterruptionTimerHard{&htim2};

	auto tim_can_timeout =
			std::shared_ptr<Clib::InterruptionTimerHard>{new(TmpMemoryPool::tim_can_timeout) Clib::InterruptionTimerHard(&htim16)};
	auto tim_monitor=
			std::shared_ptr<Clib::InterruptionTimerHard>{new(TmpMemoryPool::tim_monitor) Clib::InterruptionTimerHard(&htim17)};

	auto motor = std::array<Blib::MotorUnit,MOTOR_N>{
		Blib::MotorUnit(0,LED0_GPIO_Port,LED0_Pin,
				std::unique_ptr<Blib::IABSEncoder>(new(TmpMemoryPool::abs_enc0) Blib::AMT21xEnc(&huart5)),
				tim_can_timeout,tim_monitor),
		Blib::MotorUnit(1,LED1_GPIO_Port,LED1_Pin,
				std::unique_ptr<Blib::IABSEncoder>(new(TmpMemoryPool::abs_enc1) Blib::AMT21xEnc(&huart3)),
				tim_can_timeout,tim_monitor),
		Blib::MotorUnit(2,LED2_GPIO_Port,LED2_Pin,
				std::unique_ptr<Blib::IABSEncoder>(new(TmpMemoryPool::abs_enc2) Blib::AMT21xEnc(&hlpuart1)),
				tim_can_timeout,tim_monitor),
		Blib::MotorUnit(3,LED3_GPIO_Port,LED3_Pin,
				std::unique_ptr<Blib::IABSEncoder>(new(TmpMemoryPool::abs_enc3) Blib::AMT21xEnc(&huart2)),
				tim_can_timeout,tim_monitor)
	};

	auto flash = Blib::G4FlashRW(FLASH_BANK_2,126,0x807F000);
}

namespace be = BoardElement;

namespace Task{
	///////////////////////////////////////////////////////////////////////////
	//データの反映
	///////////////////////////////////////////////////////////////////////////
	//DataType::MDC2_IDのデータを処理
	std::optional<Clib::Protocol::DataPacket> mdc2_data_operation(const Clib::Protocol::DataPacket& dp){
		size_t motor_id = (dp.register_ID & 0x0F00)>>8;
		if(motor_id >= be::MOTOR_N){
			return std::nullopt;
		}

		if(dp.is_request){
			Clib::Protocol::DataPacket return_packet = dp;
			return_packet.data_length = 0;
			return_packet.is_request = false;
			auto w = return_packet.writer();
			if(be::motor[motor_id].id_map.get(dp.register_ID & 0x00FF, w)){
				return return_packet;
			}else{
				return std::nullopt;
			}
		}else{
			auto r = dp.reader();
			be::motor[motor_id].id_map.set(dp.register_ID & 0x00FF, r);

			return std::nullopt;
		}
	}
	std::optional<Clib::Protocol::DataPacket> common_data_operation(const Clib::Protocol::DataPacket& dp){
		Clib::Protocol::DataPacket return_packet = dp;

		switch(dp.register_ID){
		case CReg::ID_RQ:
			if(dp.is_request){
				return_packet.board_ID = be::board_id;
				return_packet.data_type = Clib::Protocol::DataType::COMMON_ID;
				return_packet.register_ID = CReg::ID_RQ;
				return_packet.is_request = false;
				return_packet.writer().write<uint8_t>(static_cast<uint8_t>(Clib::Protocol::DataType::MDC2_ID));
				return return_packet;
			}else{
				return std::nullopt;
			}
		case CReg::SAVE_PARAM:
			for(size_t i = 0; i < be::MOTOR_N; i++){
				be::motor[i].read_motor_control_param(be::init_params.param[i]);
			}
			be::init_params.never_writed = 0;
			be::flash.write(reinterpret_cast<uint8_t*>(&be::init_params), sizeof(be::MotorInitParam));
			be::led_g_sequencer.play(Blib::LEDPattern::error, true);
			return std::nullopt;
		case CReg::RESET_PARAM:
			for(size_t i = 0; i < be::MOTOR_N; i++){
				be::motor[i].write_motor_control_param(be::default_init_param);
				be::init_params.param[i] = be::default_init_param;
			}
			be::init_params.never_writed = 0;
			be::flash.write(reinterpret_cast<uint8_t*>(&be::init_params), sizeof(be::MotorInitParam));
			be::led_g_sequencer.play(Blib::LEDPattern::error, true);
			return std::nullopt;
		case CReg::EMS:
			be::led_r_sequencer.play(Blib::LEDPattern::error, true);
			HAL_Delay(500); //完全に電源が落ちるまで待機 <-旧基板では入れてたけど不要かも
			for(auto &m:be::motor){
				m.emergency_stop();
			}
			return std::nullopt;
		case CReg::RESET_EMS:
			be::led_b_sequencer.play(Blib::LEDPattern::error, true);
			HAL_Delay(100); //完全に電源が復帰するまで待機　<-旧基板では入れてたけど不要かも
			for(auto &m:be::motor){
				m.emergency_stop_release();
			}
			return std::nullopt;
		default:
			return std::nullopt;
		}
		return std::nullopt;
	}

	///////////////////////////////////////////////////////////////////////////
	//CAN/USBとの送信
	///////////////////////////////////////////////////////////////////////////
	void can_main_task(void){
		auto rx_frame = be::can_main.rx();
		if(not rx_frame.has_value()) return;

		auto dp = rx_frame.value().encode_common_data_packet();
		if(not dp.has_value()) return;

		be::led_b_sequencer.play(Blib::LEDPattern::ok);

		std::optional<Clib::Protocol::DataPacket> return_pack;
		switch(dp.value().data_type){
		case Clib::Protocol::DataType::MDC2_ID:
			if(dp.value().board_ID == be::board_id){
				return_pack = mdc2_data_operation(dp.value());
			}
			break;
		case Clib::Protocol::DataType::COMMON_ID:
			if(dp.value().board_ID == be::board_id){
				return_pack = common_data_operation(dp.value());
			}
			break;
		case Clib::Protocol::DataType::COMMON_ID_ENFORCE:
			return_pack = common_data_operation(dp.value());
			break;
		default:
			return_pack = std::nullopt;
		}

		if(return_pack.has_value()){
			Clib::CanFrame tx_frame;
			tx_frame.decode_common_data_packet(return_pack.value());
			be::can_main.tx(tx_frame);
		}
	}

	void usb_task(){
		auto rx_str = be::usb_cdc.rx();
		if(not rx_str.has_value()) return;

		Clib::CanFrame rx_frame = Clib::SLCAN::slcan_packed_to_can(rx_str.value());
		auto dp = rx_frame.encode_common_data_packet();
		if(not dp.has_value()) return;

		std::optional<Clib::Protocol::DataPacket> return_pack;
		switch(dp.value().data_type){
		case Clib::Protocol::DataType::MDC2_ID:
			if(dp.value().board_ID == be::board_id){
				be::led_b_sequencer.play(Blib::LEDPattern::ok);
				return_pack = mdc2_data_operation(dp.value());
			}
			break;
		case Clib::Protocol::DataType::COMMON_ID:
			if(dp.value().board_ID == be::board_id){
				be::led_b_sequencer.play(Blib::LEDPattern::ok);
				return_pack = common_data_operation(dp.value());
			}
			break;
		case Clib::Protocol::DataType::COMMON_ID_ENFORCE:
			be::led_b_sequencer.play(Blib::LEDPattern::ok);
			return_pack = common_data_operation(dp.value());
			break;
		default:
			return_pack = std::nullopt;
		}

		if(return_pack.has_value()){
			Clib::CanFrame tx_frame;
			tx_frame.decode_common_data_packet(return_pack.value());
			Clib::StrPack tx_str = Clib::SLCAN::can_to_slcan_packed(tx_frame);

			be::usb_cdc.tx(tx_str);
		}
	}

	///////////////////////////////////////////////////////////////////////////
	//ロボマスモタドラへの送信
	///////////////////////////////////////////////////////////////////////////
	void can_transmit_to_robomas_motor(void){
		Clib::CanFrame tx_frame;
		tx_frame.id = 0x200;

		auto w = tx_frame.writer();
		for(auto &m:be::motor){
			int16_t power = m.rm_motor.get_current_can_format();
			w.write<int16_t>(power,false);
		}
		be::can_md.tx(tx_frame);
	}
	///////////////////////////////////////////////////////////////////////////
	//VESCへの送信
	///////////////////////////////////////////////////////////////////////////
	void can_transmit_to_vesc(void){
		for(auto &m:be::motor){
			auto tx_frame = m.vesc_motor.generate_frame(m.vesc_value);
			if(tx_frame.has_value()){
				be::can_md.tx(tx_frame.value());
			}
		}
	}

	///////////////////////////////////////////////////////////////////////////
	//モニター処理
	///////////////////////////////////////////////////////////////////////////
	void monitor_task(void){
		for(size_t motor_n = 0; motor_n < be::MOTOR_N; motor_n++){
			for(auto &map_element : be::motor[motor_n].id_map.accessors_map){
				if((map_element.first < be::motor[motor_n].monitor_flags.size())
						&& be::motor[motor_n].monitor_flags.test(map_element.first)){

					Clib::Protocol::DataPacket tx_packet;
					tx_packet.register_ID = map_element.first | (motor_n << 8);
					tx_packet.board_ID = be::board_id;
					tx_packet.data_type = Clib::Protocol::DataType::MDC2_ID;

					auto writer = tx_packet.writer();
					if(map_element.second.get(writer)){
						Clib::CanFrame tx_frame;
						tx_frame.decode_common_data_packet(tx_packet);
						be::can_main.tx(tx_frame);
					}
				}
			}
		}
	}

	uint8_t read_board_id(void){
		uint8_t id = 0;
		for(int i = 0; i<4; i++){
			id |= static_cast<uint8_t>(! be::id_pins[i]()) << i;
		}
		return id;
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//メイン処理
///////////////////////////////////////////////////////////////////////////////////////////////////
extern "C"
void cppmain(void){
	//初期設定の適用
	be::flash.read(reinterpret_cast<uint8_t*>(&be::init_params), sizeof(be::MotorInitParam));
	if(be::init_params.never_writed == -1){
		for(auto &m:be::motor){
			m.write_motor_control_param(be::default_init_param);
		}
	}else{
		for(size_t i = 0; i < be::MOTOR_N; i++){
			be::motor[i].write_motor_control_param(be::init_params.param[i]);
		}
	}

	be::board_id = Task::read_board_id();

	be::led_r.set_period(1000);
	be::led_g.set_period(1000);
	be::led_b.set_period(1000);
	be::led_r.start();
	be::led_g.start();
	be::led_b.start();

	be::can_main.set_filter(0,
			static_cast<size_t>(Clib::Protocol::DataType::MDC2_ID)<<16 | (be::board_id<<16),
			0x00FF'0000,Clib::CanFilterMode::ONLY_EXT);
	be::can_main.start();

	be::can_md.set_filter_free(0,Clib::CanFilterMode::ONLY_STD);
	be::can_md.start();

	be::tim_1khz.set_task([](){
		Task::can_transmit_to_robomas_motor();

		be::led_r_sequencer.update();
		be::led_g_sequencer.update();
		be::led_b_sequencer.update();

		for(auto& m:be::motor){
			if(m.rm_motor.abs_enc){
				m.rm_motor.abs_enc->read_start();
			}
		}
	});

	be::tim_100hz.set_task(Task::can_transmit_to_vesc);

	be::tim_monitor->set_task(Task::monitor_task);

	be::tim_can_timeout->set_task([](){
		be::led_r_sequencer.play(Blib::LEDPattern::error, true);
		for(auto &m:be::motor){
			m.emergency_stop();
		}
	});

	be::tim_1khz.start_timer(1.0f/1000.0f);
	be::tim_100hz.start_timer(1.0f/100.0f);

	be::led_r_sequencer.play(Blib::LEDPattern::running);
	be::led_g_sequencer.play(Blib::LEDPattern::running);
	be::led_b_sequencer.play(Blib::LEDPattern::running);

	//メインループ
	while(1){
		be::led_g_sequencer.play(Blib::LEDPattern::running);
		Task::can_main_task();
		Task::usb_task();
		for(auto &m:be::motor){
			m.update_led_pattern();
		}
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////
//割り込み処理
///////////////////////////////////////////////////////////////////////////////////////////////////

//uart(rs485
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart5){
		be::motor[0].rm_motor.abs_enc->read_finish_task();
	}else if(huart == &huart3){
		be::motor[1].rm_motor.abs_enc->read_finish_task();
	}else if(huart == &hlpuart1){
		be::motor[2].rm_motor.abs_enc->read_finish_task();
	}else if(huart == &huart2){
		be::motor[3].rm_motor.abs_enc->read_finish_task();
	}

}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	if(huart == &huart5){

	}else if(huart == &huart3){

	}else if(huart == &hlpuart1){

	}else if(huart == &huart2){

	}
}

//メイン通信用
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
	be::can_main.rx_interrupt_task();
}

//ロボマス用
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
	be::can_md.rx_interrupt_task();
	auto rx_frame = be::can_md.rx();
	if(not rx_frame.has_value()) return;

	size_t id = rx_frame.value().id - 0x201;
	if(id <= 3){
		if(be::motor[id].rm_motor.update(rx_frame.value())){
			be::motor[id].led_sequence.update();
		}
	}
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
	if(htim == be::tim_1khz.get_handler()){
		be::tim_1khz.interrupt_task();
	}else if(htim == be::tim_monitor->get_handler()){
		be::tim_monitor->interrupt_task();
	}else if(htim == be::tim_can_timeout->get_handler()){
		be::tim_can_timeout->interrupt_task();
	}else if(htim == be::tim_100hz.get_handler()){
		be::tim_100hz.interrupt_task();
	}
}

extern "C"{
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len,100);
	return len;
}

void usb_cdc_rx_callback(const uint8_t *input,size_t size){
	be::usb_cdc.rx_interrupt_task(input, size);
}
}//extern "C"


