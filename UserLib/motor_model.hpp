/*
 * motor_model.hpp
 *
 *  Created on: Jul 15, 2025
 *      Author: gomas
 */

#ifndef MOTOR_MODEL_HPP_
#define MOTOR_MODEL_HPP_


#include "CommonLib/Math/filter.hpp"
#include "CommonLib/can_if.hpp"
#include "CommonLib/Protocol/id_defines.hpp"
#include "motor_param.hpp"

namespace BoardLib{

//モーターの伝達関数1/(sJ+D)の逆モデルsJ+D
class MotorInverceModel:public CommonLib::Math::IFilter<float>{
private:
	static constexpr float solve_coef1(float _f_sample,float _inertia, float _friction_coef){
		return _inertia*_f_sample + _friction_coef;
	}
	static constexpr float solve_coef2(float _f_sample,float _inertia){
		return -_inertia*_f_sample;
	}

	const float f_sample;
	float inertia;
	float friction_coef;

	//Y/X = (J/T+D) - J/Tz^-1
	//coef1 = (J/T+D)
	//coef2 = - J/T
	float coef1;
	float coef2;

	float prev_input = 0.0f;
	float prev_output = 0.0f;

public:
	MotorInverceModel(float _f_sample, float _inertia, float _friction_coef)
	:f_sample(_f_sample),
	 inertia(_inertia),
	 friction_coef(_friction_coef),
	 coef1(solve_coef1(_f_sample,_inertia,_friction_coef)),
	 coef2(solve_coef2(_f_sample,_inertia)){
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
		coef2 = solve_coef2(f_sample,inertia);
	}
	float get_inertia(void)const{
		return -coef2/f_sample;
	}
	float get_friction_coef(void)const{
		return coef1 + coef2;
	}

	void reset(void)override{
		prev_input = 0.0f;
		prev_output = 0.0f;
	}
};


//気が向いたら作
class VirtualRobomasMotor{
private:
	static constexpr float rad_to_rpm = 60.0f/(2*M_PI);
	static constexpr float rad_to_angle = static_cast<float>(1<<13)/(2*M_PI);
	const float update_freq;
	int id;
	MReg::RobomasMD m_type;

	CommonLib::Math::LowpassFilterBD<float> model;
	float k = 0.0f;
	float rad = 0.0f;
public:
	VirtualRobomasMotor(float _update_freq,int _id,MReg::RobomasMD _m_type,float _J,float _D)
	:update_freq(_update_freq),
	 id(_id),
	 m_type(_m_type),
	 model(update_freq,_D/(2.*M_PI*_J)),
	 k(1.0f/_D){}

	CommonLib::CanFrame operator() (int16_t _torque){
		float torque = RobomasMotorParam::robomas_value_to_torque(m_type, _torque);
		float speed = model(torque)*k;
		rad += speed * (1.0f/update_freq);

		CommonLib::CanFrame cf;
		cf.id = 0x201 + id;
		auto writer = cf.writer();
		writer.write<int16_t>(static_cast<int16_t>(rad * rad_to_angle * RobomasMotorParam::get_gear_ratio(m_type)),false);
		writer.write<int16_t>(static_cast<int16_t>(speed * rad_to_rpm * RobomasMotorParam::get_gear_ratio(m_type)),false);
		writer.write<int16_t>(static_cast<int16_t>(_torque),false);
		writer.write<int8_t>(m_type == MReg::RobomasMD::C610 ? 0 : 23);
		cf.data_length = 8;
		return cf;
	}

};

}


#endif /* MOTOR_MODEL_HPP_ */
