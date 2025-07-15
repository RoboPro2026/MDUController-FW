/*
 * encoder.hpp
 *
 *  Created on: Jun 24, 2024
 *      Author: gomas
 */

#ifndef ENCODER_HPP_
#define ENCODER_HPP_

#include "main.h"

#include <numeric>
#include <cmath>
#include <iterator>
#include <functional>

namespace CommonLib{

	class IEncoder{
	public:
		virtual int32_t get_angle(void)const = 0;
		virtual int32_t get_speed(void)const = 0;

		virtual float get_rad(void)const = 0;
		virtual float get_rad_speed(void)const = 0;
		virtual ~IEncoder(){}
	};

	//エンコーダーの連続化クラス
	class ContinuableEncoder : public IEncoder{
	private:
		const size_t resolution_bit;
		const size_t resolution;
		const size_t mask;

		//angle
		uint32_t angle = 0;
		uint32_t speed = 0;
		int32_t turn_count = 0;

		//speed
		const int32_t coef_angle_to_speed;
		int32_t angle_buff[16];
		uint32_t head = 0;
		const uint32_t head_mask = (sizeof(angle_buff)/sizeof(float) - 1);

		//angle -> rad
		float coef_angle_to_rad;

	public:
		ContinuableEncoder(size_t _resolution_bit,float update_freq,float gear_ratio = 1.0f):
			resolution_bit(_resolution_bit),
			resolution(1u<<resolution_bit),
			mask(resolution -1),
			coef_angle_to_speed(update_freq/(sizeof(angle_buff)/sizeof(int32_t))),
			coef_angle_to_rad(2*M_PI/(gear_ratio * static_cast<float>(resolution)))
			{
		}

		int32_t get_angle(void)const override{return angle;}
		int32_t get_speed(void)const override{return speed;}

		float get_rad(void)const override{return coef_angle_to_rad*static_cast<float>(get_angle());}
		float get_rad_speed(void)const override{return coef_angle_to_rad*static_cast<float>(get_speed());}

		void set_gear_ratio(float gear_ratio){
			coef_angle_to_rad = 2*M_PI/(gear_ratio * static_cast<float>(resolution));
		}
		float get_gear_ratio(void)const{
			return 2*M_PI/(coef_angle_to_rad * static_cast<float>(resolution));
		}

		virtual int32_t update(uint32_t _angle){
			int32_t new_angle = _angle&mask;

			//solve angle
			int32_t angle_top_2 = new_angle >> (resolution_bit-2);
			int32_t prev_angle_top_2 = (angle >> (resolution_bit-2))&0b11;

			if(prev_angle_top_2 == 3 && angle_top_2 == 0){
				++turn_count;
			}else if(prev_angle_top_2 == 0 && angle_top_2 == 3){
				--turn_count;
			}

			angle = new_angle + resolution*turn_count;

			//solve speed
			//LPFにしてもいいかも
			uint32_t head_tmp = head;
			head = (head + 1) & head_mask;
			angle_buff[head_tmp] = angle;
			speed = (angle_buff[head_tmp] - angle_buff[head])*coef_angle_to_speed;

			return angle;
		}

		virtual int32_t update(uint32_t _angle,uint32_t _speed){
			int32_t new_angle = _angle&mask;

			//solve angle
			int32_t angle_top = (new_angle >> (resolution_bit-1))&0b1;
			int32_t prev_angle_top = (angle >> (resolution_bit-1))&0b1;

			if(prev_angle_top == 1 && angle_top == 0 && speed > 0){
				turn_count ++;
			}else if(prev_angle_top == 0 && angle_top == 1 && speed < 0){
				turn_count --;
			}

			angle = new_angle + resolution*turn_count;

			speed = _speed;

			return angle;
		}

		void set_turn_count(int32_t _turn_count){turn_count = _turn_count;}
		int32_t get_turn_count(void)const{return turn_count;}

		virtual ~ContinuableEncoder(){}
	};


}

#endif /* ENCODER_HPP_ */
