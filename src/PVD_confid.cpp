#include "PVD_confid.h"
void PVD_config(void)
{
    delay(10);
    uint32_t PageError = 0;

    // 配置PWR
    PWR_PVDTypeDef sConfigPVD;
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_7;     // 低于2.9V触发掉电中断
    sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING; // 掉电后PVDO会置一，因此选择上升沿触发
    HAL_PWR_ConfigPVD(&sConfigPVD);           // HAL库配置PVD函数

    // 使能PVD
    HAL_PWR_EnablePVD(); // 开启掉电中断
    HAL_NVIC_EnableIRQ(PVD_IRQn);
    HAL_NVIC_SetPriority(PVD_IRQn, 0, 0); // 设置中断优先级，根据需要调整
    // int16_t readData[2];
    // FLASH_ReadData(FLASH_SAVE_ADDR, readData, sizeof(readData) / sizeof(int16_t));
    // FLASH_ReadData(FLASH_SAVE_ADDR + 4, current_location_save, sizeof(current_location_save) / sizeof(int16_t));
    // printf("max:%d,min:%d\r\n", readData[0], readData[1]);
    FLASH_ReadData(FLASH_SAVE_ADDR, dataToSave, sizeof(dataToSave) / sizeof(int16_t));

    max_angle = dataToSave[0] / 100;
    min_angle = dataToSave[1] / 100;
    current_angle = dataToSave[2] / 100;
    // TODO:读取EEPROM
    // uint16_t readData[2];
    // FLASH_ReadData(FLASH_SAVE_ADDR, readData, sizeof(readData) / sizeof(uint16_t));
    // max_angle = readData[0] / 100;
    // min_angle = readData[1] / 100;
    // current_location_save[0] = current_location_save[0] / 100;
    // Serial.printf("max:%f,min:%f\r\n", max_angle, min_angle);
    FLASH_EraseInitTypeDef FLASH_Init;
    FLASH_Init.TypeErase = FLASH_TYPEERASE_PAGES;
    FLASH_Init.PageAddress = FLASH_SAVE_ADDR;
    FLASH_Init.NbPages = 1;

    // printf("current_position_FKS = %d,current_position_ZM = %d/r/n", current_position_FKS, current_position_ZM);

    // 3.调用擦除函数
    HAL_FLASHEx_Erase(&FLASH_Init, &PageError);
    if (HAL_FLASHEx_Erase(&FLASH_Init, &PageError) != HAL_OK)
    {
        Serial.println("Flash erase failed!");
    }
    else
    {
        Serial.println("Flash erase success!");
    }
}
extern "C"
{
    void PVD_IRQHandler()
    {
        FLASH_WriteData_PVD(FLASH_SAVE_ADDR, dataToSave, sizeof(dataToSave) / sizeof(uint16_t));
        Serial.print("pvd");

        /* USER CODE END PVD_IRQn 0 */
        HAL_PWR_PVD_IRQHandler();
    }
}