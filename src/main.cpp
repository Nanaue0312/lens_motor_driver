#include "ctrl_mang.h"
#include "fmotor_driver.h"
#include "utools.h"
#include <Arduino.h>

void setup() {
  // sys_nvic_set_vector_table(FLASH_BASE, 0x4000);
  Serial.begin(115200);

  UTINFO("Follow Focus Motor Starting...");
  bool device_is_ok = FMotorDriver::instance().begin();
  CtrlMang::instance().begin();
  if (device_is_ok) {
    CtrlMang::instance().set_device_state(DeviceState::RUNNING);
  } else {
    CtrlMang::instance().set_device_state(DeviceState::ERROR);
  }

  FMotorDriver::instance().monitor_init(Serial);
  // PVD_config();

  UTINFO("CtrlMang Starting...");

  // CtrlMang::instance().set_device_state(DeviceState::PAIRING);

  // FMotorDriver::instance().calibration();
  UTINFO("Follow Focus Motor Started.");

  utcollab::Task(&FMotorDriver::run_forever, &FMotorDriver::instance())
      .detach();

  vTaskStartScheduler();

  UTFATAL("Insufficient RAM");
  while (1)
    ;
}

/// @brief 主循环
/// @warning 请不要在loop中调用任何函数
void loop() {
  // not used
}
