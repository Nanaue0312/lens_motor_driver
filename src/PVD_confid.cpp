#include "PVD_confid.h"

// 添加函数声明
void save_motor_position();

void PVD_config(void)
{
    delay(10);

    // 配置PWR
    PWR_PVDTypeDef sConfigPVD;
    sConfigPVD.PVDLevel = PWR_PVDLEVEL_7;     // 低于2.9V触发掉电中断
    sConfigPVD.Mode = PWR_PVD_MODE_IT_RISING; // 掉电后PVDO会置一，因此选择上升沿触发
    HAL_PWR_ConfigPVD(&sConfigPVD);           // HAL库配置PVD函数

    // 使能PVD
    HAL_PWR_EnablePVD(); // 开启掉电中断
    HAL_NVIC_EnableIRQ(PVD_IRQn);
    HAL_NVIC_SetPriority(PVD_IRQn, 0, 0); // 设置中断优先级，根据需要调整
}

extern "C"
{
    void PVD_IRQHandler()
    {
        // 使用现有的保存函数保存电机位置
        save_motor_position();
        
        /* USER CODE END PVD_IRQn 0 */
        HAL_PWR_PVD_IRQHandler();
    }
}