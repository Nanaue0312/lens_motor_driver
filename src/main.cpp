#include <Arduino.h>
#include "utools.h"
#include "fmotor_driver.h"
#include "ctrl_mang.h"

void setup()
{
    // sys_nvic_set_vector_table(FLASH_BASE, 0x4000);
    Serial.begin(115200);

    UTINFO("Follow Focus Motor Starting...");
    FMotorDriver::instance().begin();
    // PVD_config();

    UTINFO("CtrlMang Starting...");
    CtrlMang::instance().begin();
    // CtrlMang::instance().set_device_state(DeviceState::PAIRING);

    // FMotorDriver::instance().calibration();
    UTINFO("Follow Focus Motor Started.");

    utcollab::Task(&FMotorDriver::run_forever, &FMotorDriver::instance()).detach();

    vTaskStartScheduler();

    UTFATAL("Insufficient RAM");
    while (1)
        ;
}

/// @brief 主循环
/// @warning 请不要在loop中调用任何函数
void loop()
{
    // not used
}
