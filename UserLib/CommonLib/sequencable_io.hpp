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

#include <concepts>

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
	template<typename T>
	concept Sequencable = std::derived_from<T, IPWM> || std::is_same_v<T, GPIO>;

	template<Sequencable T>
	class Sequencer{
	private:
		const Note *playing_pattern = nullptr;
		uint32_t pattern_count = 0;
		uint32_t interval_count = 0;

		inline void set_value(float v){
			if constexpr(std::derived_from<T, IPWM>){
				io(0.0f);
			}else{
				io(v>0.0f);
			}
		}

	public:
		//TODO:templateを使わなくて良い方法を考える
		T io;

		Sequencer(T&& _io):
			io(std::move(_io)){
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

			set_value(playing_pattern[pattern_count].value);
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

					set_value(0.0f);

					return;
				}
				interval_count = playing_pattern[pattern_count].interval;
				set_value(playing_pattern[pattern_count].value);
			}
		}

		void output_not_force(float val){ //何らかのシーケンスを実行中の場合書き込まない
			if(not is_playing()){
				set_value(val);
			}
		}
	};
}





#endif /* SEQUENCABLE_IO_HPP_ */
