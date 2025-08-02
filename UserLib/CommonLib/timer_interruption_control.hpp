/*
 * timer_interruption_control.hpp
 *
 *  Created on: Jul 11, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_TIMER_INTERRUPTION_CONTROL_HPP_
#define COMMONLIB_TIMER_INTERRUPTION_CONTROL_HPP_

#include "main.h"

#ifdef HAL_TIM_MODULE_ENABLED

#include "timer_helper.hpp"
#include <functional>

namespace CommonLib{

//なぜかタイマー割込みを有効にした瞬間も割り込みが発生するのでその対策も兼ねたクラス
class InterruptionTimerHard{
private:
	TIM_HandleTypeDef *tim;
	bool first_interrupt_flag = false;

	std::function<void(void)> task = nullptr;
public:
	InterruptionTimerHard(TIM_HandleTypeDef *_tim,std::function<void(void)> _task = nullptr)
	:tim(_tim),task(_task){}

	bool start_timer(float period_s){ //秒単位で指定,0入力で停止
		uint32_t freq = TimerHelper::get_timer_clock_freq(tim->Instance);
		uint32_t period = (period_s * static_cast<float>(freq));
		if((period == 0) || (period_s < 0.0f)){
			HAL_TIM_Base_Stop_IT(tim);
			first_interrupt_flag = false;
			return false;
		}else{
			__HAL_TIM_SET_AUTORELOAD(tim,period-1);
			__HAL_TIM_SET_COUNTER(tim,0);
			//手動でcnt=0としているのでARPEの設定は不要

			if(HAL_TIM_Base_GetState(tim) == HAL_TIM_STATE_READY){
				HAL_TIM_Base_Start_IT(tim);
				return true;
			}else{
				return false;
			}
		}
	}

	void stop_timer(void){
		HAL_TIM_Base_Stop_IT(tim);
		first_interrupt_flag = false;
	}

	float get_timer_period(void)const{
		if(HAL_TIM_Base_GetState(tim) == HAL_TIM_STATE_BUSY){
			uint32_t freq = TimerHelper::get_timer_clock_freq(tim->Instance);
			return static_cast<float>(__HAL_TIM_GET_AUTORELOAD(tim)+1)/freq;
		}else{
			return 0;
		}
	}

	void set_task(std::function<void(void)> f){
		task = f;
	}

	TIM_HandleTypeDef *get_handler(void)const{
		return tim;
	}

	void reset_timer_count(void){
		__HAL_TIM_SET_COUNTER(tim,0);
	}

	//タイマ割込み関数内で呼び出す
	void interrupt_task(void){
		if(!first_interrupt_flag){
			first_interrupt_flag = true;
		}else if(task != nullptr){
			task();
		}
	}
};
}


#endif //HAL_TIM_MODULE_ENABLED


#endif /* COMMONLIB_TIMER_INTERRUPTION_CONTROL_HPP_ */
