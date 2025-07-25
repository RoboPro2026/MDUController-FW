/*
 * programmable_pwm.hpp
 *
 *  Created on: Jun 27, 2024
 *      Author: gomas
 */

#ifndef SEQUENCABLE_IO_HPP_
#define SEQUENCABLE_IO_HPP_

#include "pwm.hpp"
#include "gpio.hpp"

#include "main.h"

#include <functional>


namespace CommonLib{
	struct Note{
		float value;
		uint32_t interval;
	};
	inline constexpr Note end_of_io_sequence{0.0f,0};
}

inline bool operator==(const CommonLib::Note& s1,const CommonLib::Note& s2){
	return (s1.value == s2.value) && (s1.interval == s2.interval);
}

namespace CommonLib{

	class Sequencer{
	private:
		const Note *playing_pattern = nullptr;
		uint32_t pattern_count = 0;
		uint32_t interval_count = 0;

		std::function<void(float)> f;

	public:

		Sequencer(std::function<void(float)> _f):f(_f){
		}

		//pattern:実行するシーケンス，force:すでに別のシーケンスが走っている場合に上書きして実行するか
		bool play(const Note *pattern,bool force = false){
			if(is_playing() && not force){
				return false;
			}
			playing_pattern = pattern;
			pattern_count = 0;
			interval_count = 0;

			interval_count = playing_pattern[pattern_count].interval;

			f(playing_pattern[pattern_count].value);
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

					f(0.0f);

					return;
				}
				interval_count = playing_pattern[pattern_count].interval;
				f(playing_pattern[pattern_count].value);
			}
		}

		void output_not_force(float val){ //何らかのシーケンスを実行中の場合書き込まない
			if(not is_playing()){
				f(val);
			}
		}
	};
}





#endif /* SEQUENCABLE_IO_HPP_ */
