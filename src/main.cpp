#include <Arduino.h>
#include <functional>

#include "motor_fun.h"
#include "crc16.h"
#include "PVD_confid.h"
#include "utools.h"

// HardwareSerial Serial1(PA_2, PA_3); // 定义 Serial1，使用 PA2 作为TX，PA3 作为RX
void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset);

float target_angle_1 = 0;
void setup()
{
    // sys_nvic_set_vector_table(FLASH_BASE, 0x4000);
    Serial.begin(115200);
    utlog::bind_print(static_cast<size_t (HardwareSerial::*)(const char[])>(&HardwareSerial::print), &Serial);
    utlog::set_log_level_all(true);

    motor_init();
    PVD_config();

    UTINFO("max_angle:", max_angle, "\tmin_angle:", min_angle, "\tcurrent_angle:", current_angle);

    // motor.sensor_offset = current_angle;
    // xTaskCreate(TaskCalibration, (const portCHAR *)"Calibration", 128, NULL, 1, NULL);
    // xTaskCreate(TaskRecUart, (const portCHAR *)"RecUart", 1024, NULL, 1, NULL);
    utcollab::Task(TaskCalibration, nullptr).detach(128);
    utcollab::Task(TaskRecUart, nullptr).detach(1024);

    UTINFO("Starting scheduler");
    // start scheduler
    vTaskStartScheduler();

    UTINFO("setup done");
}

void loop()
{
    // motor_work(target_angle_rec_uart);
    motor_work(received_frm_data, fabs(received_slope_f));
}

/// @brief 设置系统 NVIC 向量表的位置
/// @details NVIC 的向量表重定位到新的地址。这是通过设置 SCB 的 VTOR 寄存器实现的，
/// @details 其中向量表的基地址由 baseaddr 参数给出，偏移量由 offset 参数给出。偏移量的低9位将被保留，
/// @details 因为这是 VTOR 寄存器的要求。
///
/// @param baseaddr 向量表的基地址
/// @param offset 向量表相对于基地址的偏移量
void sys_nvic_set_vector_table(uint32_t baseaddr, uint32_t offset)
{
    // 设置NVIC的向量表偏移寄存器,VTOR低9位保留,即[8:0]保留
    SCB->VTOR = baseaddr | (offset & (uint32_t)0xFFFFFE00);
}