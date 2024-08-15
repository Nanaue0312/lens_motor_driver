#ifndef __FLASH_H__
#define __FLASH_H__
#include "Arduino.h"
#include "string.h"
#include "stm32f3xx.h"
#define STM32_FLASH_SIZE 128        // STM32F302CB的Flash大小为128KB
#define STM32_SECTOR_SIZE 1024      // STM32F302CB的Flash扇区大小为2KB
#define STM32_FLASH_BASE 0x08000000 // STM32 FLASH起始地址
// 写入的FLASH地址，这里为从倒数第一个扇区地址开始写
#define FLASH_SAVE_ADDR STM32_FLASH_BASE + STM32_SECTOR_SIZE * 127

void FLASH_WriteData(int32_t FLASH_Addr, int16_t *FLASH_Data, int16_t Size);
uint16_t FLASH_ReadHalfWord(uint32_t faddr);
void FLASH_ReadData(int32_t ReadAddr, int16_t *pBuffer, int16_t NumToRead);
void FLASH_WriteData_PVD(int32_t FLASH_Addr, int16_t *FLASH_Data, int16_t Size);
#endif
