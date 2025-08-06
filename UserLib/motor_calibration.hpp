/*
 * motor_calibration.hpp
 *
 *  Created on: Jul 25, 2025
 *      Author: gomas
 */

#ifndef MOTOR_CALIBRATION_HPP_
#define MOTOR_CALIBRATION_HPP_

#include "CommonLib/Math/pid.hpp"

#include <utility>
#include <cmath>

namespace BoardLib{
class CalibrationManager{
private:
	const int measurement_n;
	const float update_period;
	float on_torque;

	const int start_up_time;
	const int on_hold_time;
	const int off_time;

	enum class State{
		ACCELERATION,
		DECELERATION,
		OFF,
	};

	State state = State::ACCELERATION;
	int loop_cnt = 0;
	int cnt = 0;
	float max_spd = 0.0f;

	float J = 0.0f;
	float D = 0.0f;
	float J_ave = 0.0f;
	float D_ave = 0.0f;

	bool error = false;

	CommonLib::Math::PIController pi;
	CommonLib::Math::LowpassFilterBD<float> lpf;

public:
	//測定回数，測定周波数，印加トルク，収束するまでの目安時間
	CalibrationManager(int _measurement_n = 4,float update_freq = 1000.0f,float _on_torque = 0.1f,float settling_time = 5.0f)
	:measurement_n(_measurement_n),
	 update_period(1.0f/update_freq),
	 on_torque(_on_torque),
	 start_up_time(static_cast<int>(settling_time*update_freq*0.02f)),
	 on_hold_time(static_cast<int>(settling_time*update_freq*1.0f)),
	 off_time(static_cast<int>(settling_time*update_freq*1.0f)),
	 pi(CommonLib::Math::PIBuilder(update_freq).set_gain(0.5, 0.1).set_limit(_on_torque).build()),
	 lpf(update_freq,50.0f){
	}

	//[モーターに印加するトルク，キャリブレーション処理を継続するか]
	std::pair<float,bool> calibration(float spd,float trq){
		switch(state){
		case State::ACCELERATION:
			error = false;
			cnt ++;
			if(cnt > on_hold_time){
				D = lpf.get()/spd;
				if(D < 0.0f){
					error = true;
				}
				max_spd = spd;
				cnt = 0;
				state = State::DECELERATION;
				return std::pair<float,bool>{0.0f,true};
			}else{
				float commnad_trq = pi(10.0f,spd);
				lpf(commnad_trq);
				return std::pair<float,bool>{commnad_trq,true};
			}
		case State::DECELERATION:
			cnt ++;
			if(abs(spd) < abs(max_spd*(1.0/M_E))){
				J = (static_cast<float>(cnt)*update_period)*D;
				cnt = 0;
				state = State::OFF;
			}
			return std::pair<float,bool>{0.0f,true};
		case State::OFF:
			cnt ++;
			if(cnt > off_time){//十分減速するまで待機
				cnt = 0;
				J_ave += J;
				D_ave += D;

				loop_cnt ++;
				state = State::ACCELERATION;
				if(loop_cnt >= measurement_n || error){ //規定回数測定
					loop_cnt = 0;
					J_ave /= measurement_n;
					D_ave /= measurement_n;
					return std::pair<float,bool>{0.0f,false};
				}else{ //測定継続
					on_torque *= -1.0f;
					pi.reset();
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
	bool is_failed(void)const{
		return error;
	}
	void reset(void){
		state = State::ACCELERATION;
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
