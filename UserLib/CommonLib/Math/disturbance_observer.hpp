/*
 * disturbance_observer.hpp
 *
 *  Created on: Jul 13, 2025
 *      Author: gomas
 */

#ifndef COMMONLIB_DISTURBANCE_OBSERVER_HPP_
#define COMMONLIB_DISTURBANCE_OBSERVER_HPP_

#include "filter.hpp"

#include <memory>

//TODO:開発と動作確認
namespace CommonLib::Math{

class DisturbanceObserver{
private:

	float prev_controller_output = 0.0f;
	float prev_observed_disturbance = 0.0f;

	CommonLib::Math::BiquadFilter<float> lpf;

public:
	std::unique_ptr<CommonLib::Math::IFilter<float>> inverse_model; //制御対象の逆モデル

	DisturbanceObserver(float operation_freq,std::unique_ptr<CommonLib::Math::HighpassFilterBD<float>> _inverse_model,float lpf_cutoff_freq)
	:lpf(operation_freq,lpf_cutoff_freq,1.0f),
	inverse_model(std::move(_inverse_model))
	 {}

	float observe_disturbance(float feedback_val,float controller_output){
		float observed_disturbance = lpf((*inverse_model)(feedback_val) - (prev_controller_output - prev_observed_disturbance));

		prev_controller_output = controller_output;
		prev_observed_disturbance = observed_disturbance;
		return observed_disturbance;
	}

};
}



#endif /* COMMONLIB_DISTURBANCE_OBSERVER_HPP_ */
