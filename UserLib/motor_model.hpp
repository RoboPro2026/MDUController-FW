/*
 * motor_model.hpp
 *
 *  Created on: Jul 15, 2025
 *      Author: gomas
 */

#ifndef MOTOR_MODEL_HPP_
#define MOTOR_MODEL_HPP_


#include "CommonLib/Math/filter.hpp"

namespace BoardLib{

//モーターの伝達関数1/(sJ+D)の逆モデルsJ+D
class MotorInverceModel:public CommonLib::Math::IFilter<float>{
private:
	const float f_sample;
	float inertia;
	float friction_coef;
	float prev_input;
	float prev_output;

	//Y/X = (J/T+D) - J/Tz^-1
	//coef1 = (J/T+D)
	//coef2 = - J/T
	float coef1 = 0;
	float coef2 = 0;
	static constexpr float solve_coef1(float _f_sample,float _inertia, float _friction_coef){
		return _inertia*_f_sample + _friction_coef;
	}
	static constexpr float solve_coef2(float _f_sample,float _inertia){
		return -_inertia*_f_sample;
	}
public:
	MotorInverceModel(float _f_sample, float _inertia, float _friction_coef)
	:f_sample(_f_sample),
	 inertia(_inertia),
	 friction_coef(_friction_coef){

	}

	float operator() (float input)override{
		prev_output = coef1*input + coef2*prev_input;
		prev_input = input;
		return prev_output;
	}
	float get(void)const{
		return prev_output;
	}

	void set_inertia(float _inertia){
		inertia = _inertia;
		coef1 = solve_coef1(f_sample,inertia,friction_coef);
		coef2 = solve_coef2(f_sample,inertia);
	}
	void set_friction_coef(float _friction_coef){
		friction_coef = _friction_coef;
		coef1 = solve_coef1(f_sample,inertia,friction_coef);
	}

	void reset(void){
		prev_input = 0.0f;
		prev_output = 0.0f;
	}
};
}


#endif /* MOTOR_MODEL_HPP_ */
