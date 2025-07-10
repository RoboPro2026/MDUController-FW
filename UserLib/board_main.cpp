/*
 * board_main.cpp
 *
 *  Created on: Jun 30, 2025
 *      Author: gomas
 */
#include "main.h"

#include "AMT212.hpp"

#include <array>


extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

extern UART_HandleTypeDef hlpuart1;
extern UART_HandleTypeDef huart5;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;

extern TIM_HandleTypeDef htim1;


namespace BoardElement{
	auto encs = std::array<SabaneLib::AMT21xState,4>{
		SabaneLib::AMT21xState(&huart5,0x54,1000.0f),
		SabaneLib::AMT21xState(&huart3,0x54,1000.0f),
		SabaneLib::AMT21xState(&hlpuart1,0x54,1000.0f),
		SabaneLib::AMT21xState(&huart2,0x54,1000.0f),
	};

}


//割り込み関数たち
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == BoardElement::encs[0].get_handler()){
		BoardElement::encs[0].tx_interrupt_task();
	}else if(huart == BoardElement::encs[1].get_handler()){
		BoardElement::encs[1].tx_interrupt_task();
	}else if(huart == BoardElement::encs[2].get_handler()){
		BoardElement::encs[2].tx_interrupt_task();
	}else if(huart == BoardElement::encs[3].get_handler()){
		BoardElement::encs[3].tx_interrupt_task();
	}
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == BoardElement::encs[0].get_handler()){
		BoardElement::encs[0].rx_interrupt_task();
	}else if(huart == BoardElement::encs[1].get_handler()){
		BoardElement::encs[1].rx_interrupt_task();
	}else if(huart == BoardElement::encs[2].get_handler()){
		BoardElement::encs[2].rx_interrupt_task();
	}else if(huart == BoardElement::encs[3].get_handler()){
		BoardElement::encs[3].rx_interrupt_task();
	}
}

extern "C"{
void cppmain(void){
	for(auto &e: BoardElement::encs){
		e.request_position();
	}
	HAL_Delay(100);

}
}



