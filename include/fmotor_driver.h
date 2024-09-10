#ifndef __FMOTOR_DRIVER_H__
#define __FMOTOR_DRIVER_H__

#include "utools.h"
#include <SimpleFOC.h>
#include "kth78xx.h"

class FMotorDriver : public utpattern::Singleton<FMotorDriver>
{
public:
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
        // __motor.motion_downsample = 2; // 下采样配置，每调用n次 __motor.move() 才执行一次运动控制

        // // foc current control parameters (Arduino UNO/Mega)
        // __motor.PID_current_q.P = 1;
        // __motor.PID_current_q.I = 0;
        // __motor.PID_current_d.P = 1;
        // __motor.PID_current_d.I = 0;
        // __motor.LPF_current_q.Tf = 1;
        // __motor.LPF_current_d.Tf = 1;
        __motor.foc_modulation = FOCModulationType::SpaceVectorPWM;
        // __motor.target = 2;
        // velocity loop PID
        __motor.PID_velocity.P = 0.2;
        __motor.PID_velocity.I = 0.2;
        __motor.PID_velocity.D = 0.001;
        __motor.PID_velocity.output_ramp = 1000.0;
        __motor.PID_velocity.limit = 1.5;
        // Low pass filtering time constant
        __motor.LPF_velocity.Tf = 0.2;

        // angle loop PID
        __motor.P_angle.P = 10.0;
        __motor.P_angle.I = 0.0;
        __motor.P_angle.D = 0.0;
        __motor.P_angle.output_ramp = 10000.0;
        __motor.P_angle.limit = 200.0;
        // Low pass filtering time constant
        __motor.LPF_angle.Tf = 0.0;

        // setting the limits
        // maximal velocity of the position control
        __motor.velocity_limit = 200; // rad/s - default 20
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
        UTINFO("Motor ready.");
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
        utcollab::Task(&FMotorDriver::__calibration, this, 20.0, 1000 * 60).detach(256);
    }

    /// @brief 设定目标角度
    /// @param angle 目标角度
    /// @return 目标角度
    inline float set_target_angle(float angle)
    {
        __target_angle = utmath::clamp(angle, __angle_range.first, __angle_range.second);
        return __target_angle;
    }

    /// @brief 运动到归一化位置，会自动映射到真实位置
    /// @param normalized_position [0, 1.0]，归一化位置
    /// @return 真实映射的位置值
    inline float set_target_normalized_position(float normalized_position)
    {
        __target_angle = utmath::linear_map(
            normalized_position,
            0.0, 1.0,
            __angle_range.first, __angle_range.second);
        return __target_angle;
    }

    /// @brief 运动到目标角度
    /// @note 调用频率大于1kHz
    inline void handle_loop()
    {
        __motor.loopFOC();
        __motor.move(__target_angle);
    }

private:
    // 电机角度范围
    std::pair<float, float> __angle_range{0, 45};

    float current_angle = 0;
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

    bool __calibrating_flag{false}; // 正校准标志位

    /// @brief 校准电机
    /// @param velocity_limit 校准速度限制
    void __calibration(const float velocity_limit = 20.0, const int64_t timeout = 1000 * 60)
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
            __target_angle += 20; // 设置目标角度
            if (std::abs(__sensor.getAngle() - last_angle) < 0.1)
            {
                cal_range.second = __sensor.getAngle() - 1.5;
                goto CALIBRATION_SETP2; // 跳转到阶段2
            }
            last_angle = __sensor.getAngle(); // 获取当前角度
            utcollab::Task::sleep_for(utime::to_std_ms(100));
        }
        goto CALIBRATION_TIMEOUT; // 超时跳转

        // 阶段2：电机反转
    CALIBRATION_SETP2: // 跳转设置2
        UTTRACE("Calibration Motor Backward.");
        start_time = utime::boot_ts();                  // 记录开始时间
        __target_angle = cal_range.second - 30;         // 设置目标角度
        while (utime::boot_ts() - start_time < timeout) // 最多执行1分钟
        {
            __target_angle -= 20; // 设置目标角度
            if (std::abs(__sensor.getAngle() - last_angle) < 0.1)
            {
                cal_range.first = __sensor.getAngle() + 1.5;
                goto CALIBRATION_END; // 跳转到阶段3
            }
            last_angle = __sensor.getAngle(); // 获取当前角度
            utcollab::Task::sleep_for(utime::to_std_ms(100));
        }

        // 校准出现超时
    CALIBRATION_TIMEOUT:
        UTTRACE("Calibration Motor Timeout.");
        goto CALIBRATION_CLEAR;

        // 校准结束
    CALIBRATION_END:
        __angle_range.swap(cal_range); // 交换角度范围
        UTTRACE("Calibration Motor Range of Motion is: [", __angle_range.first, ",", __angle_range.second, "]");
        UTTRACE("Calibration Motor End.");

        // 清除校准标志等
    CALIBRATION_CLEAR:
        __calibrating_flag = false;
        __target_angle = __sensor.getAngle(); // 让电机停止工作
    }
};

#endif // __FMOTOR_DRIVER_H__
