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

namespace CommonLib{
	struct Note{
		float power;
		uint32_t interval;
	};
	inline constexpr Note end_of_io_sequence{0.0f,0};
}

inline bool operator==(const CommonLib::Note& s1,const CommonLib::Note& s2){
	return (s1.power == s2.power) && (s1.interval == s2.interval);
}

namespace CommonLib{
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
		std::unique_ptr<T> io;

		SequencableIO(std::unique_ptr<T> _io):
			io(std::move(_io)){
		}

		bool play(const Note *pattern,bool force = true){
			if(is_playing() && not force){
				return false;
			}
			playing_pattern = pattern;
			pattern_count = 0;
			interval_count = 0;

			interval_count = playing_pattern[pattern_count].interval;

			(*io)(playing_pattern[pattern_count].power);
			return true;
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

				if(playing_pattern[pattern_count] == end_of_io_sequence){
					playing_pattern = nullptr;

					set_duty(0.0f);

					return;
				}
				interval_count = playing_pattern[pattern_count].interval;
				set_duty(playing_pattern[pattern_count].power);
			}
		}

		void out_not_force(float val){ //何らかのシーケンスを実行中の場合書き込まない
			if(not is_playing()){
				set_duty(val);
			}
		}
	};
}





#endif /* SEQUENCABLE_IO_HPP_ */
