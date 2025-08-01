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

extern USBD_HandleTypeDef hUsbDeviceFS;

namespace Clib = CommonLib;
namespace Blib = BoardLib;

namespace BoardElement{
	size_t board_id = 0x0;

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

	auto id_pins = std::array<Clib::GPIO,4>{
		Clib::GPIO{ID0_GPIO_Port,ID0_Pin},
		Clib::GPIO{ID1_GPIO_Port,ID1_Pin},
		Clib::GPIO{ID2_GPIO_Port,ID2_Pin},
		Clib::GPIO{ID3_GPIO_Port,ID3_Pin}
	};

	auto tim_1khz = Clib::InterruptionTimerHard{&htim15};

	//TODO:iocファイルの設定が古いかもしれないので確認
	auto tim_can_timeout = std::shared_ptr<Clib::InterruptionTimerHard>{new(TmpMemoryPool::tim_can_timeout) Clib::InterruptionTimerHard(&htim16)};
	auto tim_monitor= std::shared_ptr<Clib::InterruptionTimerHard>{new(TmpMemoryPool::tim_monitor) Clib::InterruptionTimerHard(&htim17)};

	auto motor = std::array<Blib::MotorUnit,4>{
		Blib::MotorUnit(0,LED0_GPIO_Port,LED0_Pin,std::unique_ptr<Blib::IABSEncoder>(new(TmpMemoryPool::abs_enc0) Blib::AMT21xEnc(&huart5))),
		Blib::MotorUnit(1,LED1_GPIO_Port,LED1_Pin,std::unique_ptr<Blib::IABSEncoder>(new(TmpMemoryPool::abs_enc1) Blib::AMT21xEnc(&huart3))),
		Blib::MotorUnit(2,LED2_GPIO_Port,LED2_Pin,std::unique_ptr<Blib::IABSEncoder>(new(TmpMemoryPool::abs_enc2) Blib::AMT21xEnc(&hlpuart1))),
		Blib::MotorUnit(3,LED3_GPIO_Port,LED3_Pin,std::unique_ptr<Blib::IABSEncoder>(new(TmpMemoryPool::abs_enc3) Blib::AMT21xEnc(&huart2)))
	};
}

namespace be = BoardElement;

namespace Task{
	///////////////////////////////////////////////////////////////////////////
	//データの反映
	///////////////////////////////////////////////////////////////////////////
	//DataType::MDC2_IDのデータを処理
	std::optional<Clib::Protocol::DataPacket> mdc2_data_operation(const Clib::Protocol::DataPacket& dp){
		size_t motor_id = (dp.register_ID & 0x0F00)>>8;
		if(motor_id >= 4){
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
		//TODO:動作チェック
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
		case CReg::EMS:
			for(auto &m:be::motor){
				m.emergency_stop();
			}
			return std::nullopt;
		case CReg::RESET_EMS:
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
	be::board_id = Task::read_board_id();

	be::led_r.start();
	be::led_g.start();
	be::led_b.start();

	be::can_main.set_filter(0, 0x0040'0000 | be::board_id, 0x00FF'0000,Clib::CanFilterMode::ONLY_EXT);
	be::can_main.start();
	be::can_md.set_filter_free(0,Clib::CanFilterMode::ONLY_STD);
	be::can_md.start();

	be::tim_1khz.set_task([](){
		Task::can_transmit_to_robomas_motor();
		Task::can_transmit_to_vesc();

		be::led_r_sequencer.update();
		be::led_g_sequencer.update();
		be::led_b_sequencer.update();

		for(auto& m:be::motor){
			if(m.rm_motor.abs_enc){
				m.rm_motor.abs_enc->read_start();
			}
		}
	});

	be::tim_1khz.start_timer(1.0f/1000.0f);

	be::led_r_sequencer.play(Blib::LEDPattern::setting);
	be::led_g_sequencer.play(Blib::LEDPattern::setting);
	be::led_b_sequencer.play(Blib::LEDPattern::setting);

	while(1){
		be::led_g_sequencer.play(Blib::LEDPattern::test);
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
	be::led_b_sequencer.play(Blib::LEDPattern::ok);
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
	}
}

extern "C"{
int _write(int file, char *ptr, int len) {
	HAL_UART_Transmit(&huart2, (uint8_t*) ptr, len,100);
	return len;
}

void usb_cdc_rx_callback(const uint8_t *input,size_t size){
	be::usb_cdc.rx_interrupt_task(input, size);
	be::led_b_sequencer.play(Blib::LEDPattern::ok);
}
}//extern "C"


