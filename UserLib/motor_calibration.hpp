/*
 * motor_calibration.hpp
 *
 *  Created on: Jul 25, 2025
 *      Author: gomas
 */

#ifndef MOTOR_CALIBRATION_HPP_
#define MOTOR_CALIBRATION_HPP_

#include <utility>
#include <cmath>

namespace BoardLib{
class CalibrationManager{
private:
	static constexpr int start_up_time = 100;
	static constexpr int on_hold_time = 5000;
	static constexpr int off_time = 3000;
	static constexpr int measurement_n = 6;

	float on_torque = 0.1;

	enum class State{
		START_UP,
		ON_HOLD,
		DECELERATION,
		OFF,
	};

	State state = State::START_UP;
	int loop_cnt = 0;
	int cnt = 0;
	float max_spd = 0.0f;

	float J = 0.0f;
	float D = 0.0f;
	float J_ave = 0.0f;
	float D_ave = 0.0f;
public:
	//キャリブレーションする際は1kHzで呼び出し
	std::pair<float,bool> calibration(float spd,float trq){
		switch(state){
		case State::START_UP:
			cnt ++;
			if(cnt > start_up_time){
				cnt = 0;
				state = State::ON_HOLD;
				if(loop_cnt == 0){ //初回なら合計をゼロリセット
					J_ave = 0.0f;
					D_ave = 0.0f;
				}
			}
			return std::pair<float,bool>{on_torque * 5,true};
		case State::ON_HOLD:
			cnt ++;
			if(cnt > on_hold_time){
				D = trq/spd;
				max_spd = spd;
				cnt = 0;
				state = State::DECELERATION;
				return std::pair<float,bool>{0.0f,true};
			}else{
				return std::pair<float,bool>{on_torque,true};
			}
		case State::DECELERATION:
			cnt ++;
			if(abs(spd) < abs(max_spd*(1-0.632f))){
				J = (static_cast<float>(cnt)/1000.f)*D;
				cnt = 0;
				state = State::OFF;
			}
			return std::pair<float,bool>{0.0f,true};
		case State::OFF:
			cnt ++;
			if(cnt > off_time){//十分減速するまで待機
				cnt = 0;
				if(J < 0.0f || D < 0.0f){ //値がおかしい場合もう一度測定
					loop_cnt --;
				}else{
					J_ave += J;
					D_ave += D;
				}

				loop_cnt ++;
				state = State::START_UP;
				if(loop_cnt > measurement_n){ //規定回数測定
					loop_cnt = 0;
					J_ave /= measurement_n;
					D_ave /= measurement_n;
					return std::pair<float,bool>{0.0f,false};
				}else{ //測定継続
					on_torque *= -1.0f;
					return std::pair<float,bool>{0.0f,true};
				}
			}else{
				return std::pair<float,bool>{0.0f,true};
			}
		}
		return std::pair<float,bool>{0.0f,false};
	}

	float get_inertia(void)const{
		return J_ave;
	}
	float get_friction_coef(void)const{
		return D_ave;
	}
	void reset(void){
		state = State::START_UP;
		loop_cnt = 0;
		cnt = 0;
		max_spd = 0.0f;

		J = 0.0f;
		D = 0.0f;
		J_ave = 0.0f;
		D_ave = 0.0f;
	}
};
}



#endif /* MOTOR_CALIBRATION_HPP_ */
