/*
 * programmable_pwm.hpp
 *
 *  Created on: Jun 27, 2024
 *      Author: gomas
 */

#ifndef SEQUENCABLE_IO_HPP_
#define SEQUENCABLE_IO_HPP_

#include "pwm.hpp"
#include "main.h"

#include <memory>
#include "gpio.hpp"

namespace SabaneLib{
	struct Note{
		float power;
		uint32_t interval;
	};
}

inline bool operator==(const SabaneLib::Note& s1,const SabaneLib::Note& s2){
	return (s1.power == s2.power) && (s1.interval == s2.interval);
}

namespace SabaneLib{
	template<typename T>
	concept Sequencable = std::derived_from<T, IPWM> || std::is_same_v<T, GPIO>;

	template<Sequencable T>
	class SequencableIO{
	private:
		const Note *playing_pattern = nullptr;
		uint32_t pattern_count = 0;
		uint32_t interval_count = 0;

		inline void set_duty(float v){
			if constexpr(std::derived_from<T, IPWM>){
				(*io)(0.0f);
			}else{
				if(v > 0.0f){
					(*io)(true);
				}else{
					(*io)(false);
				}
			}
		}

	public:
		static constexpr Note end_of_pwm_sequence{0.0f,0};
		std::unique_ptr<T> io;

		SequencableIO(std::unique_ptr<T> _io):
			io(std::move(_io)){
		}

		void play(const Note *pattern){
			playing_pattern = pattern;
			pattern_count = 0;
			interval_count = 0;

			interval_count = playing_pattern[pattern_count].interval;

			(*io)(playing_pattern[pattern_count].power);
		}

		bool is_playing(void){
			return playing_pattern!=nullptr;
		}

		void update(void){

			if(playing_pattern == nullptr){
				return;
			}
			interval_count  --;
			if(interval_count <= 0){
				++pattern_count;

				if(playing_pattern[pattern_count] == end_of_pwm_sequence){
					playing_pattern = nullptr;

					set_duty(0.0f);

					return;
				}
				interval_count = playing_pattern[pattern_count].interval;
				set_duty(playing_pattern[pattern_count].power);
			}
		}

		void out_weak(float val){
			if(not is_playing()){
				set_duty(val);
			}
		}
	};
}





#endif /* SEQUENCABLE_IO_HPP_ */
