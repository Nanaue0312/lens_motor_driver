#include "ctrl_mang.h"
#include "fmotor_driver.h"
#include "utools.h"
#include <Arduino.h>
#include "motor_comm.h"
#include "simple_protocol_impl.h"
#include "simple_protocol_tpl.h"
#include "app_version.h"
#include "EEPROM.h"
#include "PVD_confid.h"

SimpleProtocolImpl *sprotocol_report = dynamic_cast<SimpleProtocolImpl *>(new SimpleProtocolTpl<1, 1, true, 32, 0>({0xA5, 0xAB}));
lens_motor_data_t motor_report_data;
broadcast_data_t received_data;
void recveive_handler();
// 上报状态任务
void report_status_task();
void save_motor_position();
void setup()
{
  // sys_nvic_set_vector_table(FLASH_BASE, 0x4000);
  Serial.begin(115200);
  // auto print_func = [](const char *message)
  // {
  //   Serial.print(message);
  // };
  // utlog::bind_print(print_func);
  // utlog::start_async(512);
  UTINFO("Follow Focus Motor Starting...");
  // float last_position = 0;
  // EEPROM.get(0, last_position);
  bool device_is_ok = FMotorDriver::instance().begin();
  CtrlMang::instance().begin();
  if (device_is_ok)
  {
    // FMotorDriver::instance().target_angle = last_position;
    CtrlMang::instance().set_device_state(DeviceState::RUNNING);
  }
  else
  {
    CtrlMang::instance().set_device_state(DeviceState::ERROR);
  }

  // FMotorDriver::instance().monitor_init(Serial);
  // PVD_config();

  UTINFO("CtrlMang Starting...");

  // CtrlMang::instance().set_device_state(DeviceState::PAIRING);

  UTINFO("Follow Focus Motor Started.");
  utcollab::Task(&FMotorDriver::run_forever, &FMotorDriver::instance()).detach();
  utcollab::Task(recveive_handler).detach();
  utcollab::Task(report_status_task).detach();
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

void handle_calibration(uint8_t cal_flag, float target_value, float hitwall)
{
  switch (cal_flag)
  {
  case 0:
    if (CtrlMang::instance().device_state != DeviceState::CALIBRATION)
    {
      FMotorDriver::instance().calibration();
    }
    break;
  case 2:
    if (target_value > hitwall && hitwall != 0)
    {
      FMotorDriver::instance().target_angle = hitwall;
    }
    else 
    {
      FMotorDriver::instance().target_angle = target_value;
    }
  case 3:

    break;
    // case 1,3: do nothing
  }
}

void handle_speed_setting(uint8_t speed_flag)
{
  const float SPEED_SLOW = 0.007f;
  const float SPEED_MEDIUM = 0.01f;
  const float SPEED_FAST = 0.013f;

  switch (speed_flag)
  {
  case 1:
    FMotorDriver::instance().set_speed(SPEED_SLOW);
    break;
  case 2:
    FMotorDriver::instance().set_speed(SPEED_MEDIUM);
    break;
  case 3:
    FMotorDriver::instance().set_speed(SPEED_FAST);
    break;
  }
}

void process_mode_data(MotorFuncMode mode, const broadcast_data_t *data)
{
  switch (mode)
  {
  case MotorFuncMode::ZOOM:
    handle_calibration(data->motorflag_zoom_status, data->zoom, 1);
    handle_speed_setting(data->motorflag_zoom_speed);
    break;
  case MotorFuncMode::IRIS:
    handle_calibration(data->motorflag_iris_status, data->iris, 1);
    handle_speed_setting(data->motorflag_iris_speed);
    break;
  case MotorFuncMode::FOCUS:
    handle_calibration(data->motorflag_focus_status, data->focus, data->hitwall);
    handle_speed_setting(data->motorflag_focus_speed);
    break;
  }
}

void recveive_handler()
{
  UTDEBUG("Serial recv handler started.");
  const size_t BUFFER_SIZE = 32;
  uint8_t recv_buffer[BUFFER_SIZE];

  while (true)
  {
    auto recv_len = Serial.available();
    auto &motor = FMotorDriver::instance();

    if (recv_len > 0 && !motor.move_flag)
    {
      motor.move_flag = true;

      Serial.readBytes(recv_buffer, recv_len);
      auto frms{sprotocol_recv->push_back(recv_buffer, recv_len).parse_all()};

      for (const auto &frm : frms)
      {
        if (frm.is_valid())
        {
          // received_data = mootreinterpret_cast<broadcast_data_t *>(const_cast<uint8_t *>(frm.data.data()));
          memcpy(&received_data, frm.data.data(), sizeof(broadcast_data_t));

          auto current_mode = CtrlMang::instance().get_motor_func_mode();
          UTDEBUG("current mode: ", current_mode);

          process_mode_data(current_mode, &received_data);
        }
      }
    }
    // if (motor.move_flag) {
    //     save_motor_position();
    // }
    utcollab::Task::sleep_for(5);
  }
}

void report_status_task()
{
  while (true)
  {
    memset(&motor_report_data, 0, sizeof(lens_motor_data_t));
    motor_report_data.funcode = (uint8_t)(CtrlMang::instance().get_motor_func_mode());
    motor_report_data.ts = millis();
    motor_report_data.mac = AppVersion::mac();
    motor_report_data.value = FMotorDriver::instance().get_current_target();
    if (CtrlMang::instance().device_state == DeviceState::CALIBRATION)
      motor_report_data.flag_status_cal = 1;
    else if (CtrlMang::instance().device_state == DeviceState::CALIBRATION_OK)
    {
      motor_report_data.flag_status_cal = 2;
    }

    auto frame{sprotocol_report->make_packer(1, 32)};
    frame.push_back(&motor_report_data, sizeof(lens_motor_data_t)).end_pack();
    Serial.write(frame().data(), frame().size());
    utcollab::Task::sleep_for(15);
  }
}

void save_motor_position()
{
  float current_pos = FMotorDriver::instance().get_current_target();
  // STM32不需要commit，直接使用put即可
  EEPROM.put(0, current_pos);
}
