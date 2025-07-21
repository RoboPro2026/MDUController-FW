/*
 * motor_param.hpp
 *
 *  Created on: Jul 19, 2025
 *      Author: gomas
 */

#ifndef MOTOR_PARAM_HPP_
#define MOTOR_PARAM_HPP_

#include "CommonLib/Protocol/id_defines.hpp"
#include <cstdint>
#include <algorithm>

namespace BoardLib{
class RobomasMotorParam{
	//ロボマスモーターのモーターパラメータ
	//TODO:値の確認
	static constexpr float torque_coef_inv[2] = {1.0f/0.18f, 1.0f/0.3f}; //トルク定数の逆数[A/(N/m)]
	static constexpr float gear_ratio[2] = {36.0f,3591.0f/187.0f};
	static constexpr float current_limit[2] = {10.0f,20.0f};
	static constexpr float current_to_robomas_param_coef[2] = {1000.0f,16384.0f/20.0f};
public:
	static constexpr float get_gear_ratio(MReg::RobomasMD m_type){
		return gear_ratio[static_cast<size_t>(m_type)];
	}
	static constexpr int16_t torque_to_robomas_value(MReg::RobomasMD m_type, float torque){
		int index = static_cast<size_t>(m_type);
		float current = std::clamp(torque*torque_coef_inv[index], -current_limit[index], current_limit[index]);
		return static_cast<int16_t>(current * current_to_robomas_param_coef[index]);
	}
};



}//BoardLib

#endif /* MOTOR_PARAM_HPP_ */
