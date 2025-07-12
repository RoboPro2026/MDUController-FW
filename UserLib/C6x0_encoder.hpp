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

namespace BoardLib{
	class C6x0Enc:public CommonLib::ContinuableEncoder{
	private:
		static constexpr size_t enc_resolution = 13;

		float current = 0.0f;
		float temperature = 0;
	public:
		C6x0Enc(float feedbuck_freq = 1000.0f)
		:CommonLib::ContinuableEncoder(13,feedbuck_freq){
		}

		bool update_by_can_msg(CommonLib::CanFrame frame){
			if(frame.is_ext_id || frame.is_remote || frame.data_length != 8 || !(0x200&frame.id)){
				return false;
			}
			uint16_t angle = frame.data[0]<<8 | frame.data[1];
			int16_t angle_speed = frame.data[2]<<8 | frame.data[3];

			update(angle,angle_speed);

			current = static_cast<float>(frame.data[4]<<8 | frame.data[5]);
			temperature = static_cast<float>(frame.data[6]);
			return true;
		}

		float get_temperature(void)const{
			return temperature;
		}
		float get_current(void)const{
			return current;
		}
	};
}



#endif /* C6X0_ENCODER_HPP_ */
