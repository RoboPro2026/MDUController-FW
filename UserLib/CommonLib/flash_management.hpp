/*
 * flash_management.hpp
 *
 *  Created on: Jul 26, 2025
 *      Author: gomas
 */

#ifndef FLASH_MANAGEMENT_HPP_
#define FLASH_MANAGEMENT_HPP_

#include "main.h"
#include <string.h>

//リンカの設定などをわすれないこと
//https://rt-net.jp/mobility/archives/25755
namespace CommonLib{

enum class FlashState:int{
	RESET = -1,
	WRITED = 0
};

class IFrashRW{
public:
	virtual void write(const uint8_t *data,size_t len) = 0;
	virtual void read(uint8_t *data,size_t len) = 0;
};

#ifdef STM32G4xx_HAL_H
#ifdef HAL_FLASH_MODULE_ENABLED

class G4FlashRW:public IFrashRW{
	const uint32_t bank;
	const uint32_t page;
	const uint32_t start_addr;
public:

	G4FlashRW(uint32_t _bank,uint32_t _page,uint32_t _start_addr):bank(_bank),page(_page),start_addr(_start_addr){}

	void write(const uint8_t *data,size_t len)override{
		HAL_FLASH_Unlock();
		FLASH_EraseInitTypeDef erase;

		erase.TypeErase = FLASH_TYPEERASE_PAGES;
		erase.Banks = bank;
		erase.Page = page;
		erase.NbPages = 1;

		uint32_t error_val = 0;
		HAL_FLASHEx_Erase(&erase,&error_val);

		for(size_t i = 0; i < len; i+=8){
			HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD,start_addr + i, *(uint64_t*)(void*)&data[i]);
		}

		HAL_FLASH_Lock();
	}
	void read(uint8_t *data,size_t len)override{
		memcpy(data,(void*)start_addr,len);
	}
};
#endif //HAL_FLASH_MODULE_ENABLED
#endif //STM32G4xx_HAL_H
}//namespace CommonLib


#endif /* FLASH_MANAGEMENT_HPP_ */
