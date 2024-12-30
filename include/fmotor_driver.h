#ifndef __FMOTOR_DRIVER_H__
#define __FMOTOR_DRIVER_H__
#include "simple_protocol_impl.h"
#include "simple_protocol_tpl.h"
#include "broadcast_data.h"
SimpleProtocolImpl *sprotocol_recv = dynamic_cast<SimpleProtocolImpl *>(new SimpleProtocolTpl<1, 1, true, 32, 0>({0xA5, 0xAB}));

#include "kth78xx.h"
#include "utools.h"
#include <SimpleFOC.h>

void doMotor(char *cmd);
void doTarget(char *cmd);
class FMotorDriver : public utpattern::Singleton<FMotorDriver>
{
public:
  float target_angle = 0;
  bool move_flag{false};

  FMotorDriver()
  {
    // 1 位置检测传感器初始化
    __sensor.init();
    UTINFO("kth7812 sensor ready.");
    // 连接电机和传感器
    __motor.linkSensor(&__sensor);

    // 2 驱动程序配置
    __driver.voltage_power_supply = 12; // 电源电压 [V]
    __driver.voltage_limit = 24;
    __driver.pwm_frequency = 20000;
    auto re_dr = __driver.init();
    if (re_dr != 1)
    {
      UTERROR("driver init failed. status:", re_dr);
      return;
    }
    UTINFO("driver ready.");
    __motor.linkDriver(&__driver);

    // 3 电流检测传感器初始化
    __current_sense.linkDriver(&__driver);
    auto re_cs = __current_sense.init();
    if (re_cs != 1)
    {
      UTERROR("current sense init failed. status:", re_cs);
      return;
    }
    UTINFO("current sense initialized success.");

    // 4 电机环初始化
    __motor.linkCurrentSense(&__current_sense);

    __motor.torque_controller = TorqueControlType::voltage;
    __motor.controller = MotionControlType::angle;
    // __motor.motion_downsample = 2; // 下采样配置，每调用n次 __motor.move()
    // 才执行一次运动控制

    // // foc current control parameters (Arduino UNO/Mega)
    // __motor.PID_current_q.P = 0.6;
    // __motor.PID_current_q.I = 0;
    // __motor.PID_current_d.P = 0;
    // __motor.PID_current_d.I = 0;
    // __motor.LPF_current_q.Tf = 0.4;
    // __motor.LPF_current_d.Tf = 0.5;
    __motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
    // __motor.target = 2;
    // velocity loop PID
    // __motor.PID_velocity.P = 0.3;
    // __motor.PID_velocity.I = 0.3;
    // __motor.PID_velocity.D = 0.001;
    // todo 调整还是存在卡顿
    __motor.PID_velocity.P = 0.01;
    __motor.PID_velocity.I = 0.001;
    __motor.PID_velocity.D = 0;
    __motor.PID_velocity.output_ramp =1000;
    __motor.PID_velocity.limit = 1.5;
    // Low pass filtering time constant
    __motor.LPF_velocity.Tf = 0.2;

    // angle loop PID
    // __motor.P_angle = angle_pid_0;
    __motor.P_angle.P = 80.0;
    __motor.P_angle.I = 0.01;
    __motor.P_angle.D = 0;
    __motor.P_angle.output_ramp = 10000.0;
    __motor.P_angle.limit = 200.0;
    // // Low pass filtering time constant
    __motor.LPF_angle.Tf = 0;

    // setting the limits
    // maximal velocity of the position control
    __motor.velocity_limit = 150; // rad/s - default 20
    __motor.voltage_limit = 10.0;
    __motor.current_limit = 1.5;

    __motor.phase_resistance = 5;
    __motor.modulation_centered = 1.0;
  }

  virtual ~FMotorDriver() = default;

  bool begin()
  {
    __motor.init();
    auto re{__motor.initFOC()};
    if(re != 1)
    {
      UTERROR("motor init failed. status:", re);
    }
    return re == 1;
  }

  /// @brief 启动电机校准
  void calibration()
  {
    if (__calibrating_flag)
    {
      UTINFO("Motor is already calibrating.");
      return;
    }
    utcollab::Task(&FMotorDriver::__calibration, this, 20.0, 1000 * 60)
        .detach(256);
  }

  /// @brief 设定目标角度
  /// @param angle 目标角度
  /// @return 目标角度
  inline float set_target_angle(float angle)
  {
    __target_angle =
        utmath::clamp(angle, __angle_range.first, __angle_range.second);
    return __target_angle;
  }

  /// @brief 运动到归一化位置，会自动映射到真实位置
  /// @param normalized_position [0, 1.0]，归一化位置
  /// @return 真实映射的位置值
  inline float set_target_normalized_position(float normalized_position)
  {
    __target_angle =
        utmath::linear_map(normalized_position, 0.0, 1.0, __angle_range.first,
                           __angle_range.second);
    return __target_angle;
  }

  /// @brief 运动到目标角度
  /// @note 调用频率大于1kHz
  inline void handle()
  {
    // static float last_angle{__target_angle};
    __motor.loopFOC();
    // __target_angle = __motor.target;
    __motor.move(__target_angle);

    // if (utmath::is_close_abs(__motor.shaft_angle, __target_angle, 0.008f)) {
    //     if (blend_factor > 0.0f) {
    //         blend_factor -= 0.001f; // 如果需要恢复，可以逐渐减少
    //         blend_factor
    //     }
    //     __motor.P_angle.P = (1.0f - blend_factor) * __motor.P_angle.P +
    //                         blend_factor * angle_pid_0.P;
    //     __motor.P_angle.I = (1.0f - blend_factor) * __motor.P_angle.I +
    //                         blend_factor * angle_pid_0.I;
    //     __motor.P_angle.D = (1.0f - blend_factor) * __motor.P_angle.D +
    //                         blend_factor * angle_pid_0.D;
    // } else if (is_out_percent(last_angle, __target_angle,
    // __motor.shaft_angle, 0.7)) {
    //     if (blend_factor < 1.0f) {
    //         blend_factor += 0.001f; // 逐步自增，调节这个增量以控制过渡速度
    //     }
    //     __motor.P_angle.P = (1.0f - blend_factor) * __motor.P_angle.P +
    //                         blend_factor * angle_pid_60.P;
    //     __motor.P_angle.I = (1.0f - blend_factor) * __motor.P_angle.I +
    //                         blend_factor * angle_pid_60.I;
    //     __motor.P_angle.D = (1.0f - blend_factor) * __motor.P_angle.D +
    //                         blend_factor * angle_pid_60.D;
    // }
    // if (utmath::is_close_abs(__motor.shaft_angle, __target_angle, 0.01f)) {
    //   last_angle = __target_angle;
    // }
  }

  /// @brief 运行主循环
  /// @note 调用频率大于1kHz
  /// @warning 调用后会在此处阻塞，直到程序结束，请让此函数在单独的线程中运行
  void run_forever()
  {
    static bool first_run{true};
    String rx_str;

    static float last_target_angle = 0.0f;
    if (!first_run)
    {
      UTWARN("motor driver already run.");
      return;
    }
    first_run = false;
    UTINFO("motor driver run forever.");
    while (true) // 永久循环
    {
      
      set_target_normalized_position(target_angle); // 设置目标位置
      handle();
      move_flag = false;
      // 调试部分
      // __motor.monitor();
      // __command.run();
    }
  }
  /// @brief 初始化commander和monitor，用于simple foc studio 调参
  /// @param serial
  void monitor_init(HardwareSerial &serial)
  {
    __command = Commander(serial);
    __command.add('M', doMotor, "motor");
    __command.add('T', doTarget, "motor target");
    __motor.useMonitoring(serial);
    __motor.monitor_downsample = 100;
  }

  BLDCMotor &get_motor() { return __motor; }
  Commander &get_command() { return __command; }
  void set_target(float target) { __target_angle = target; }
  float *get_target() { return &__target_angle; }

  /**
   * @brief 获取当前角度（弧度值）
   * @return 
   */
  float get_current_target(){
      return utmath::linear_map(__sensor.getSensorAngle() / 3.14f * 180 , 0, 1, __angle_range.first, __angle_range.second);
  }
private:
  // 电机角度范围
  std::pair<float, float> __angle_range{0, 45};

  // voltage set point variable
  float __target_angle = 0;
  float target_angle_rec_uart = 0;
  bool calibrating = false; // 校准状态标志
  bool calibration_flag = false;
  // BLDC driver instance
  BLDCDriver6PWM __driver{PA8, PA11, PA9, PA12, PA10, PB1};
  // BLDC motor instance
  BLDCMotor __motor{7, 10, 180};
  // LowsideCurrentSense构造函数
  LowsideCurrentSense __current_sense{0.001, 11, PA6, PA7, PA5};
  GenericSensor __sensor{kth7812_read_angle, kth7812_init};
  Commander __command;
  PIDController angle_pid_0{10.0f, 0.1f, 0.0f, 10000.0f, 150.0f};
  // PIDController angle_pid_60{8.0f, 0.4f, 0.0f, 10000.0f, 150.0f};
  // 判断是否切换pid
  bool pid_switched = false;
  /// @brief 检测当前位置是否大于误差
  ///
  /// @param start 起始值
  /// @param target 目标值
  /// @param cur 当前值
  /// @param percent 百分比小鼠
  /// @return true 超过百分比
  /// @return false 没超过百分比
  /*   bool is_out_percent(float start, float target, float cur, float percent)
    { if (target < start) { return cur < (start + (target - start) * percent);
      } else {
        return cur > (start + (target - start) * percent);
      }
    } */

  bool __calibrating_flag{false}; // 正校准标志位

  /// @brief 校准电机
  /// @note 用于确定电机转动的角度上限
  /// @param velocity_limit 校准速度限制
  void __calibration(const float velocity_limit = 20.0,
                     const int64_t timeout = 1000 * 60)
  {
    UTTRACE("Calibration Motor Start.");
    __calibrating_flag = true;               // 设置校准标志位
    __motor.velocity_limit = velocity_limit; // 设置电机速度限制
    float cal_angle{0.0};                    // 获取当前角度
    bool flag_direction{true};               // 方向标志
    float last_angle{0};                     // 上一次角度
    std::pair<float, float> cal_range{0, 0}; // 校准范围

    // 阶段1：电机正转
    UTTRACE("Calibration Motor Forward.");
    auto start_time{utime::boot_ts()};              // 记录开始时间
    while (utime::boot_ts() - start_time < timeout) // 最多执行1分钟
    {
      auto curr_angle = __sensor.getAngle();
      __target_angle = curr_angle + 20; // 设置目标角度
      if (std::abs(__sensor.getAngle() - last_angle) < 0.1)
      {
        cal_range.second = __sensor.getAngle() - 1.5;
        goto CALIBRATION_SETP2; // 跳转到阶段2
      }
      last_angle = curr_angle; // 获取当前角度
      utcollab::Task::sleep_for(100);
    }
    goto CALIBRATION_TIMEOUT; // 超时跳转

    // 阶段2：电机反转
  CALIBRATION_SETP2: // 跳转设置2
    UTTRACE("Calibration Motor Backward.");
    start_time = utime::boot_ts();                  // 记录开始时间
    __target_angle = cal_range.second - 30;         // 设置目标角度
    while (utime::boot_ts() - start_time < timeout) // 最多执行1分钟
    {
      auto curr_angle = __sensor.getAngle();
      __target_angle = curr_angle - 20; // 设置目标角度
      if (std::abs(curr_angle - last_angle) < 0.1)
      {
        cal_range.first = __sensor.getAngle() + 1.5;
        goto CALIBRATION_END; // 跳转到阶段3
      }
      last_angle = curr_angle; // 获取当前角度
      utcollab::Task::sleep_for(100);
    }

    // 校准出现超时
  CALIBRATION_TIMEOUT:
    UTTRACE("Calibration Motor Timeout.");
    CtrlMang::instance().set_device_state(DeviceState::CALIBRATION_ERROR);
    goto CALIBRATION_CLEAR;

    // 校准结束
  CALIBRATION_END:
    __angle_range.swap(cal_range); // 交换角度范围
    CtrlMang::instance().set_device_state(DeviceState::CALIBRATION_OK);
    UTTRACE("Calibration Motor Range of Motion is: [", __angle_range.first, ",",
            __angle_range.second, "]");
    UTTRACE("Calibration Motor End.");

    // 清除校准标志等
  CALIBRATION_CLEAR:
    __calibrating_flag = false;
    __target_angle = __sensor.getAngle(); // 让电机停止工作
  }
};
void doMotor(char *cmd)
{
  FMotorDriver::instance().get_command().motor(
      &FMotorDriver::instance().get_motor(), cmd);
}
void doTarget(char *cmd)
{
  FMotorDriver::instance().get_command().scalar(
      FMotorDriver::instance().get_target(), cmd);
} // 串口控制指令：目标值
#endif // __FMOTOR_DRIVER_H__
