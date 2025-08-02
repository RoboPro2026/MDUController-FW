/*
 * control_unit.hpp
 *
 *  Created on: Jul 28, 2025
 *      Author: gomas
 */

#ifndef CONTROL_UNIT_HPP_
#define CONTROL_UNIT_HPP_

#include "CommonLib/Protocol/id_defines.hpp"
#include "CommonLib/id_map_control.hpp"
#include "CommonLib/gpio.hpp"
#include "CommonLib/sequencer.hpp"
#include "abs_encoder.hpp"

#include "motor_control.hpp"
#include "vesc_data.hpp"
#include "LED_pattern.hpp"

#include <memory>

namespace BoardLib{

struct MotorUnit{
	bool is_active = false;
	C6x0Controller rm_motor;
	VescDataConverter vesc_motor;

	float vesc_value = 0.0f;

	CommonLib::GPIO led;
	CommonLib::Sequencer led_sequence;

	const CommonLib::Note* led_playing_pattern;

	std::shared_ptr<CommonLib::InterruptionTimerHard> timeout_tim = nullptr;
	std::shared_ptr<CommonLib::InterruptionTimerHard> monitor_tim = nullptr;

	CommonLib::IDMap id_map;

	void set_control_mode(uint8_t c_val){
		MReg::ControlMode mode = static_cast<MReg::ControlMode>(c_val&0b11);
		if(static_cast<size_t>(mode) == 0b11) return;

		MReg::RobomasMD md = static_cast<MReg::RobomasMD>((c_val >> 2) & 0b1);
		bool use_dob = ((c_val >> 4)&0b1) == 0b1;
		bool use_abs_enc = ((c_val >> 5)&0b1) == 0b1;
		bool est_m_type = ((c_val >> 5)&0b1) == 0b1;

		rm_motor.set_control_mode(mode);
		rm_motor.set_motor_type(md);
		rm_motor.use_abs_enc(use_abs_enc);
		rm_motor.use_dob(use_dob);
		rm_motor.estimate_motor_type(est_m_type);
	}

	void update_led_pattern(void){
		led_sequence.play(BoardLib::LEDPattern::led_mode_indicate[rm_motor.is_using_abs_enc() ? 1 : 0][static_cast<size_t>(rm_motor.get_control_mode())]);
	}

	uint8_t get_control_mode(void)const{
		return static_cast<uint8_t>(rm_motor.get_control_mode())
				| (static_cast<uint8_t>(rm_motor.get_motor_type()) << 2)
				| (rm_motor.is_using_dob()?0b0001'0000:0)
				| (rm_motor.is_using_abs_enc()?0b0010'0000:0);
	}

	uint8_t mode_tmp = 0;
	void emergency_stop(void){
		mode_tmp = get_control_mode();
		set_control_mode(0);
	}
	void emergency_stop_release(void){
		set_control_mode(mode_tmp);
	}

	MotorUnit(int id,GPIO_TypeDef *led_port,uint_fast16_t led_pin,std::unique_ptr<BoardLib::IABSEncoder> abs_enc):
		rm_motor(C6x0ControllerBuilder(id,MReg::RobomasMD::C610).set_abs_enc(std::move(abs_enc), false).build()),
		vesc_motor(id),
		led(led_port,led_pin),
		led_sequence([&](float v){led(v>0.0f);}),
		led_playing_pattern(BoardLib::LEDPattern::led_mode_indicate[0][0]),
		id_map(CommonLib::IDMapBuilder()
				.add(MReg::MOTOR_STATE,    CommonLib::DataAccessor::generate<bool>(&is_active))
				.add(MReg::CONTROL,        CommonLib::DataAccessor::generate<uint8_t>(
						[&](uint8_t v)mutable{set_control_mode(v);},
						[&]()->uint8_t{return get_control_mode();}))
				.add(MReg::ABS_GEAR_RATIO, CommonLib::DataAccessor::generate<bool>(
						[&](float r)mutable{rm_motor.abs_enc->set_gear_ratio(r);},
						[&]()->float{return rm_motor.abs_enc->get_gear_ratio();}))
				.add(MReg::CAL_RQ,         CommonLib::DataAccessor::generate<bool>(
						[&](bool s)mutable{rm_motor.start_calibration();},
						[&]()->bool{return rm_motor.is_calibrating();}))
				.add(MReg::LOAD_J,         CommonLib::DataAccessor::generate<float>(
						[&](float j)mutable{rm_motor.dob.inverse_model.set_inertia(j);},
						[&]()->float{return rm_motor.dob.inverse_model.get_inertia();}))
				.add(MReg::LOAD_D,         CommonLib::DataAccessor::generate<float>(
						[&](float d)mutable{rm_motor.dob.inverse_model.set_friction_coef(d);},
						[&]()->float{return rm_motor.dob.inverse_model.get_friction_coef();}))
				.add(MReg::DOB_CF,         CommonLib::DataAccessor::generate<float>(
						[&](float f)mutable{rm_motor.dob.set_lpf_cutoff_freq(f);}))

				.add(MReg::CAN_TIMEOUT,    CommonLib::DataAccessor::generate<uint16_t>(
						[&](uint16_t p)mutable{
							if(timeout_tim){
								timeout_tim->start_timer(p==0 ? -1.0f : static_cast<float>(p)*0.001);
							}
						}))

				.add(MReg::TRQ,            CommonLib::DataAccessor::generate<float>(
						[&]()->float{return rm_motor.enc.get_torque();}))
				.add(MReg::TRQ_TARGET,     CommonLib::DataAccessor::generate<float>(
						[&](float t)mutable{rm_motor.set_torque(t);},
						[&]()->float{return rm_motor.get_torque();}))

				.add(MReg::SPD,            CommonLib::DataAccessor::generate<float>(
						[&]()->float{return rm_motor.enc.get_rad_speed();}))
				.add(MReg::SPD_TARGET,     CommonLib::DataAccessor::generate<float>(
						[&](float s)mutable{rm_motor.set_target_speed_rad(s);},
						[&]()->float{return rm_motor.get_target_speed_rad();}))
				.add(MReg::TRQ_LIM,        CommonLib::DataAccessor::generate<float>(
						[&](float l)mutable{rm_motor.spd_pid.set_limit(l);},
						[&]()->float{auto [ll,lh] = rm_motor.spd_pid.get_limit(); return lh;}))
				.add(MReg::SPD_GAIN_P,     CommonLib::DataAccessor::generate<float>(
						[&](float p)mutable{rm_motor.spd_pid.set_p_gain(p);},
						[&]()->float{return  rm_motor.spd_pid.get_p_gain();}))
				.add(MReg::SPD_GAIN_I,     CommonLib::DataAccessor::generate<float>(
						[&](float i)mutable{rm_motor.spd_pid.set_i_gain(i);},
						[&]()->float{return rm_motor.spd_pid.get_i_gain();}))
				.add(MReg::SPD_GAIN_D,     CommonLib::DataAccessor::generate<float>(
						[&](float d)mutable{rm_motor.spd_pid.set_d_gain(d);},
						[&]()->float{return rm_motor.spd_pid.get_d_gain();}))

				.add(MReg::POS,            CommonLib::DataAccessor::generate<float>(
						[&](float r)mutable{rm_motor.overwrite_rad(r);},
						[&]()->float{return rm_motor.get_overwrited_rad();}))
				.add(MReg::POS_TARGET,     CommonLib::DataAccessor::generate<float>(
						[&](float s)mutable{rm_motor.set_target_rad(s);},
						[&]()->float{return rm_motor.get_target_rad();}))
				.add(MReg::SPD_LIM,        CommonLib::DataAccessor::generate<float>(
						[&](float l)mutable{rm_motor.pos_pid.set_limit(l);},
						[&]()->float{auto [ll,lh] = rm_motor.pos_pid.get_limit(); return lh;}))
				.add(MReg::POS_GAIN_P,     CommonLib::DataAccessor::generate<float>(
						[&](float p)mutable{rm_motor.pos_pid.set_p_gain(p);},
						[&]()->float{return  rm_motor.pos_pid.get_p_gain();}))
				.add(MReg::POS_GAIN_I,     CommonLib::DataAccessor::generate<float>(
						[&](float i)mutable{rm_motor.pos_pid.set_i_gain(i);},
						[&]()->float{return rm_motor.pos_pid.get_i_gain();}))
				.add(MReg::POS_GAIN_D,     CommonLib::DataAccessor::generate<float>(
						[&](float d)mutable{rm_motor.pos_pid.set_d_gain(d);},
						[&]()->float{return rm_motor.pos_pid.get_d_gain();}))

				.add(MReg::ABS_POS,        CommonLib::DataAccessor::generate<float>(
						[&]()->float{return rm_motor.abs_enc->get_rad();}))
				.add(MReg::ABS_SPD,        CommonLib::DataAccessor::generate<float>(
						[&]()->float{return rm_motor.abs_enc->get_rad_speed();}))
				.add(MReg::ABS_TURN_CNT,   CommonLib::DataAccessor::generate<int32_t>(
						[&](int32_t t)mutable{rm_motor.abs_enc->set_turn_count(t);},
						[&]()->int32_t{return rm_motor.abs_enc->get_turn_count();}))

				.add(MReg::VESC_MODE,      CommonLib::DataAccessor::generate<int8_t>(
						[&](int8_t m)mutable{vesc_motor.set_mode(static_cast<MReg::VescMode>(m));},
						[&]()->int8_t{return static_cast<uint8_t>(vesc_motor.get_mode());}))
				.add(MReg::VESC_TARGET,    CommonLib::DataAccessor::generate<float>(&vesc_value))
				
				.add(MReg::MONITOR_PERIOD, CommonLib::DataAccessor::generate<uint16_t>(
						[&](uint16_t p)mutable{
							if(monitor_tim){
								monitor_tim->start_timer(p==0 ? -1.0f : static_cast<float>(p)*0.001);
							}
						}))
				
				.build()){

		set_control_mode(0b0000'0000);
	}
};


}//namespace BoardLib



#endif /* CONTROL_UNIT_HPP_ */
