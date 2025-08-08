/*
 * AMT212.hpp
 *
 *  Created on: Jul 10, 2025
 *      Author: gomas
 */

#ifndef ABS_ENCODER_HPP_
#define ABS_ENCODER_HPP_

#include "main.h"
#include "CommonLib/encoder.hpp"

#include <unordered_map>

namespace BoardLib{

class IABSEncoder:public CommonLib::ContinuableEncoder{
public:
	IABSEncoder(size_t _resolution_bit,float update_freq,float gear_ratio = 1.0f)
	:ContinuableEncoder(_resolution_bit,update_freq,gear_ratio){
	}
	enum class Context{
		SUCCESS,
		FAILURE
	};
	virtual void init(void) = 0;
	virtual bool is_ready(void)const = 0;
	virtual bool is_dead(void)const = 0;
	virtual void read_start(void) = 0;
	virtual void read_finish_task(Context c = Context::SUCCESS) = 0;
	virtual ~IABSEncoder(){}
};

#ifdef HAL_UART_MODULE_ENABLED
class AMT21xEnc:public IABSEncoder{
private:
	static constexpr size_t enc_resolution = 14;
	UART_HandleTypeDef* const uart;

	const uint8_t enc_id;

	uint8_t enc_val[2] = {0};

	bool new_data_available;
	bool no_responce;

#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
	static std::unordered_map<UART_HandleTypeDef *,std::function<void(void)>> callbacks;
	static void callback(UART_HandleTypeDef *huart){
		auto iter = callbacks.find(huart);
		if(iter == callbacks.end()){
			//nop
		}else{
			iter->second();
		}
	}
#endif

public:
	AMT21xEnc(UART_HandleTypeDef* _uart,uint8_t _enc_id = 0x54,float update_freq = 1000.0f,float gear_ratio = 1.0f)
		:IABSEncoder(enc_resolution,update_freq,gear_ratio),
		 uart(_uart),
		 enc_id(_enc_id),
		 new_data_available(false),
		 no_responce(false){
		if(uart == nullptr){
			no_responce = true;
		}
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
		callbacks.insert(std::pair(uart,[&](){this->read_finish_task();}));
#endif
	}

	void init(void)override{
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
		HAL_UART_RegisterCallback(uart, HAL_UART_RX_COMPLETE_CB_ID,callback);
#endif
	}

	bool is_ready(void)const override{
		return new_data_available;
	}

	bool is_dead(void)const override{
		return no_responce;
	}


	//エンコーダとの通信に使用する関数群
	//read_start
	//↓
	//read_finish_task(受信完了割り込み内）
	void read_start(void)override{
		if(uart == nullptr){
			return;
		}
		HAL_UART_Transmit_IT(uart, const_cast<uint8_t*>(&enc_id),1);
		HAL_UART_Receive_IT(uart, enc_val, 2);

		if(not new_data_available){
			no_responce = true;
			return;
		}
		new_data_available = false;
	}

	//HAL_UART_RxCpltCallback内におくこと
	void read_finish_task(Context c = Context::SUCCESS)override{
		uint16_t raw_angle = enc_val[1]<<8 | enc_val[0];
		update(raw_angle);
		no_responce = false;
		new_data_available = true;
	}

	UART_HandleTypeDef* get_handler(void){
		return uart;
	}
};
#if (USE_HAL_UART_REGISTER_CALLBACKS == 1)
inline std::unordered_map<UART_HandleTypeDef *,std::function<void(void)>> AMT21xEnc::callbacks;
#endif

#endif //HAL_UART_MODULE_ENABLED


#ifdef HAL_I2C_MODULE_ENABLED
class AS5600State:public IABSEncoder{
private:
	static constexpr uint16_t as5600_id = 0x36;
	static constexpr size_t as5600_resolution = 12;

	I2C_HandleTypeDef* const i2c;

	uint8_t enc_val[2] = {0};

	const int32_t inv = 1;

	bool new_data_available;
	bool no_responce;
public:
	AS5600State(I2C_HandleTypeDef* _i2c,float update_freq,float gear_ratio = 1.0f)
		:i2c(_i2c),
		 IABSEncoder(as5600_resolution,update_freq,gear_ratio),
		 new_data_available(false),
		 no_responce(false){
	}

	bool is_ready(void)const override{
		return new_data_available;
	}

	bool is_dead(void)const override{
		return no_responce;
	}

	void read_start(void)override{
		HAL_I2C_Mem_Read_IT(i2c, as5600_id<<1, 0x0c, I2C_MEMADD_SIZE_8BIT, enc_val, 2);
		if(not new_data_available){
			no_responce = true;
			return;
		}
		new_data_available = false;
	}

	//HAL_I2C_MemRxCpltCallbackで呼び出すこと(context = 0)
	//HAL_I2C_ErrorCallbackの場合context = 1
	void read_finish_task(Context c = Context::SUCCESS)override{//通常の
		if(c == Context::SUCESS){
			uint16_t raw_angle = enc_val[0]<<8 | enc_val[1];
			update(raw_angle);
			new_data_available = true;
			no_responce = false;
		}else{
			new_data_available = false;
			no_responce = true;
		}
	}
};
#endif //HAL_I2C_MODULE_ENABLED

}// namespace BoardLib

#endif /* ABS_ENCODER_HPP_ */
