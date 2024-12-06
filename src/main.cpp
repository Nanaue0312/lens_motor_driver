#include "ctrl_mang.h"
#include "fmotor_driver.h"
#include "utools.h"
#include <Arduino.h>
#include "motor_comm.h"
#include "simple_protocol_impl.h"
#include "simple_protocol_tpl.h"
SimpleProtocolImpl *sprotocol_report = dynamic_cast<SimpleProtocolImpl *>(new SimpleProtocolTpl<1, 1, true, 32, 0>({0xA5, 0xA5}));
lens_motor_data_t *motor_report_data;
void recveive_handler();
// 上报状态任务
void report_status_task();
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
  bool device_is_ok = FMotorDriver::instance().begin();

  CtrlMang::instance().begin();
  if (device_is_ok)
  {
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

  // FMotorDriver::instance().calibration();
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

void recveive_handler()
{
  UTDEBUG("Serial recv handler started.");
  while (true)
  {
    auto recv_len = Serial.available();
    // 检查串口是否有足够的数据可读
    if (recv_len > 0 && !FMotorDriver::instance().move_flag)
    {

      uint8_t recv_buffer[32];
      FMotorDriver::instance().move_flag = true; // 设置移动标志

      // 从串口读取数据到结构体中
      Serial.readBytes(recv_buffer, recv_len);
      auto frms{sprotocol_recv->push_back(recv_buffer, recv_len).parse_all()};
      for (auto &frm : frms)
      {
        if (frm.is_valid())
        {
          broadcast_data_t *received_data = reinterpret_cast<broadcast_data_t *>(const_cast<uint8_t *>(frm.data.data()));
          // 获取当前模式
          MotorFuncMode current_mode = CtrlMang::instance().get_motor_func_mode();
          UTDEBUG("current mode: ", current_mode);
          // 根据当前模式提取数据
          if (current_mode == MotorFuncMode::ZOOM)
          {
            switch (received_data->motorflag_zoom)
            {
            case 0:
              FMotorDriver::instance().target_angle = received_data->zoom; // 提取 zoom 值
              break;
            case 1:
              FMotorDriver::instance().calibration();
              break;
            default:
              break;
            }
          }
          else if (current_mode == MotorFuncMode::IRIS)
          {
            switch (received_data->motorflag_iris)
            {
            case 0:
              FMotorDriver::instance().target_angle = received_data->iris; // 提取 iris 值
              break;
            case 1:
              FMotorDriver::instance().calibration();
              break;
            default:
              break;
            }
          }
          else if (current_mode == MotorFuncMode::FOCUS)
          {
            switch (received_data->motorflag_focus)
            {
            case 0:
              FMotorDriver::instance().target_angle = received_data->focus; // 提取 focus 值
              break;
            case 1:
              FMotorDriver::instance().calibration();
              break;
            default:
              break;
            }
          }
        }
      }
    }
    utcollab::Task::sleep_for(1);
  }
}

void report_status_task()
{
  while (true)
  {
    // memset(motor_report_data, 0, sizeof(lens_motor_data_t));
    // motor_report_data->funcode = (uint8_t)(CtrlMang::instance().get_motor_func_mode());
    // motor_report_data->ts = millis();
    // motor_report_data->mac = 0x12345678;
    // motor_report_data->value = CtrlMang::instance().get_target_normalized_postion();
    // motor_report_data->flag_status = (uint8_t)(CtrlMang::instance().device_state);
    auto frame{sprotocol_report->make_packer(1, 32)};
    frame.push_back(motor_report_data, sizeof(broadcast_data_t)).end_pack();
    Serial.write(frame().data(), frame().size());
    utcollab::Task::sleep_for(100);
  }
}