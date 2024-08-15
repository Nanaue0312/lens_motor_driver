#include "bsp_internal_flash.h"
#include "stm32f3xx_hal_flash.h"
/*从指定地址开始写入数据*/
// FLASH_Addr:起始地址
// FLASH_Data:写入数据指针(写入的数据为16位，至少都要16位)
// Size:写入数据长度

void FLASH_WriteData(int32_t FLASH_Addr, int16_t *FLASH_Data, int16_t Size)
{
    // 解锁FLASH
    HAL_FLASH_Unlock();

    // 擦除扇区
    FLASH_EraseInitTypeDef eraseInitStruct = {0};
    eraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
    eraseInitStruct.PageAddress = FLASH_Addr;
    eraseInitStruct.NbPages = 1;

    uint32_t pageError = 0;
    HAL_FLASHEx_Erase(&eraseInitStruct, &pageError);

    // 对FLASH烧写
    uint16_t TempBuf = 0;
    for (uint16_t i = 0; i < Size; i++)
    {
        TempBuf = *(FLASH_Data + i);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_Addr + (i * 2), TempBuf);
    }
    // 锁住FLASH
    HAL_FLASH_Lock();
    int16_t readData[2];
    FLASH_ReadData(FLASH_SAVE_ADDR, readData, sizeof(readData) / sizeof(int16_t));

    printf("max:%d,min:%d\r\n", readData[0], readData[1]);
}

void FLASH_WriteData_PVD(int32_t FLASH_Addr, int16_t *FLASH_Data, int16_t Size)
{
    // 解锁FLASH
    HAL_FLASH_Unlock();
    // 对FLASH烧写
    uint16_t TempBuf = 0;
    for (uint16_t i = 0; i < Size; i++)
    {
        TempBuf = *(FLASH_Data + i);
        HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, FLASH_Addr + (i * 2), TempBuf);
    }
    // 锁住FLASH
    HAL_FLASH_Lock();
}
// 从指定地址读出数据
// faddr：地址
uint16_t FLASH_ReadHalfWord(uint32_t faddr)
{
    return *(__IO uint16_t *)faddr;
}

// 从指定地址开始读出指定长度的数据
// ReadAddr:起始指针
// pBuffer:数据指针
// NumToRead:大小（至少半字(16位)数）
void FLASH_ReadData(int32_t ReadAddr, int16_t *pBuffer, int16_t NumToRead)
{
    for (int16_t i = 0; i < NumToRead; i++)
    {
        pBuffer[i] = FLASH_ReadHalfWord(ReadAddr);
        ReadAddr += 2;
    }
}