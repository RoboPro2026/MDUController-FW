/*
 * C6x0_encoder.hpp
 *
 *  Created on: Jul 12, 2025
 *      Author: gomas
 */

#ifndef C6X0_ENCODER_HPP_
#define C6X0_ENCODER_HPP_

#include "CommonLib/can_if.hpp"
#include "CommonLib/encoder.hpp"
#include "motor_param.hpp"
#include "CommonLib/Math/filter.hpp"

namespace BoardLib{
	class C6x0Enc:public CommonLib::ContinuableEncoder{
	private:
		static constexpr size_t enc_resolution = 13;
		static constexpr int32_t rpm_to_angle_speed = (1<<13)/60.0f;

		MReg::RobomasMD m_type;
		int16_t current = 0;
		int8_t temperature = 0;

		CommonLib::Math::LowpassFilterBD<float> i_lpf;
	public:
		C6x0Enc(MReg::RobomasMD _m_type,float update_freq = 1000.0f,float i_lpf_cutoff = 50.0f)
		:CommonLib::ContinuableEncoder(13,update_freq,RobomasMotorParam::get_gear_ratio(_m_type)),
		 m_type(_m_type),
		 i_lpf(update_freq,i_lpf_cutoff){
		}

		void set_motor_type(MReg::RobomasMD _m_type){
			CommonLib::ContinuableEncoder::set_gear_ratio(RobomasMotorParam::get_gear_ratio(_m_type));
			m_type = _m_type;
		}

		bool update_by_can_msg(CommonLib::CanFrame frame){
			if(frame.is_ext_id || frame.is_remote || frame.data_length != 8 || !(0x200&frame.id)){
				return false;
			}
			uint16_t angle = frame.data[0]<<8 | frame.data[1];
			int32_t angle_speed = static_cast<int16_t>(frame.data[2]<<8 | frame.data[3])*rpm_to_angle_speed;

			update(angle,angle_speed);

			current = static_cast<int16_t>(frame.data[4]<<8 | frame.data[5]);
			i_lpf(RobomasMotorParam::robomas_value_to_torque(m_type, current));
			temperature = frame.data[6];
			return true;
		}

		int8_t get_temperature(void)const{
			return temperature;
		}
		float get_torque(void)const{
			return i_lpf.get();//;RobomasMotorParam::robomas_value_to_torque(m_type, current);
		}
		int16_t get_current(void)const{
			return current;
		}
	};
}



#endif /* C6X0_ENCODER_HPP_ */
