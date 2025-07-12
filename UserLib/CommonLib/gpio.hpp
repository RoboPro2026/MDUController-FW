/*
 * GPIO.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef GPIO_HPP_
#define GPIO_HPP_

#include "main.h"

namespace CommonLib{

class GPIO{
private:
	GPIO_TypeDef* const port;
	const uint_fast16_t pin;
public:
	GPIO(GPIO_TypeDef *_port,uint_fast16_t _pin)
		: port(_port),pin(_pin){
	}

	enum class Mode{
		INPUT  = 0b00,
		OUTPUT = 0b01,
		FUNCTION =  0b10,
		ANALOG = 0b11 //reset
	};
	void set_io_mode(Mode m){
		//LLのLL_GPIO_SetPinModeを移植
		MODIFY_REG(port->MODER, (GPIO_MODER_MODE0 << (POSITION_VAL(pin) * 2U)), (static_cast<int>(m) << (POSITION_VAL(pin) * 2U)));
	}

	bool operator() (void)const{
		return (port->IDR & pin) != 0;
	}

	void operator() (bool state){
		//state==0なら16bitシフトしてRSRR.BRに書き込み->リセット
		//state==1ならシフトせずBSRR.BSに書き込み->セット
		port->BSRR = pin << (16*static_cast<int>(! state));
	}

	void toggle(void){
		if((*this)()){
			(*this)(false);
		}else{
			(*this)(true);
		}
	}
};

}



#endif /* GPIO_HPP_ */
