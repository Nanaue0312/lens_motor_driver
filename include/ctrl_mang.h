/// @brief 用于处理用户的操作指令，如控制电机运动，并生成相应的调用指令，调用电机驱动程序

#ifndef __CTRL_MANG_H__
#define __CTRL_MANG_H__

#include "utools.h"
#include <Arduino.h>
#include "state_led.h"
#include "keyboard.h"
#include "keymap.h"

/// @brief 状态LED定义
#define STATE_LED1_PIN PA0 // LED_G
#define STATE_LED2_PIN PA1 // LED_B
#define STATE_LED3_PIN PA4 // LED_R

/// @brief 功能按键定义
#define SHIFT_PIN PB0 // KEYBOARD

/// @brief 设备的状态
enum class DeviceState
{
    NONE,        // 未初始化
    INIT,        // 初始化中
    PAIRING,     // 配对中
    CALIBRATION, // 校准中
    RUNNING,     // 运行中
    ERROR,       // 错误
};

/// @brief 电机控制模式
enum class MotorFuncMode
{
    FOCUS, // 对焦模式
    ZOOM,  // 缩放模式
    IRIS,  // 光圈模式
};

class CtrlMang : public utpattern::Singleton<CtrlMang>
{
public:
    void begin()
    {
        // 初始化LED灯
        pinMode(STATE_LED1_PIN, OUTPUT);
        pinMode(STATE_LED2_PIN, OUTPUT);
        pinMode(STATE_LED3_PIN, OUTPUT);
        set_device_state(DeviceState::INIT);
        // UTINFO("Init state led ok.");
        // 初始化键盘
        pinMode(SHIFT_PIN, INPUT_PULLUP);
        Keyboard::instance()
            .bind_event_handle(&CtrlMang::__on_key_event, &CtrlMang::instance())
            .add_key(Keyboard::Key(KEY_SHIFT, Keyboard::KeyAttr(), LOW, digitalRead, SHIFT_PIN));
        utcollab::Task(&CtrlMang::__update_key_loop, &CtrlMang::instance()).detach();
        // UTINFO("Init keyboard ok.");
        // TODO:增加数据加载功能
    }

    /// @brief 设置电机运动范围
    /// @return 保存在eeprom中
    std::pair<float, float> get_motor_motion_range()
    {
        return __motor_motion_range;
    }

    /// @brief 获取目标控制角度
    /// @return 返回目录控制角度，该参数为归一化值，范围在0~1之间
    float get_target_normalized_postion()
    {
        return __target_normalized_postion;
    }

    StateLED state_led1{std::bind(digitalWrite, STATE_LED1_PIN, LOW), std::bind(digitalWrite, STATE_LED1_PIN, HIGH)};
    StateLED state_led2{std::bind(digitalWrite, STATE_LED2_PIN, LOW), std::bind(digitalWrite, STATE_LED2_PIN, HIGH)};
    StateLED state_led3{std::bind(digitalWrite, STATE_LED3_PIN, LOW), std::bind(digitalWrite, STATE_LED3_PIN, HIGH)};

    /// @brief 设备状态
    DeviceState device_state{DeviceState::NONE};

    /// @brief 电机功能模式
    MotorFuncMode motor_func_mode{MotorFuncMode::FOCUS};
    MotorFuncMode get_motor_func_mode() const
    {
        return motor_func_mode;
    }

    /// @brief 设置设备的状态
    /// @param state 设备状态
    void set_device_state(DeviceState state)
    {
        if (device_state != state)
        {
            device_state = state;
            switch (state)
            {
            case DeviceState::INIT:
                utcollab::Task(&CtrlMang::__state_led_init, this).detach(1024, 0, 4, -1);
                break;
            case DeviceState::PAIRING:
                utcollab::Task(&CtrlMang::__state_led_pairing, this).detach(1024, 0, 4, -1);
                break;
            case DeviceState::CALIBRATION:
                utcollab::Task(&CtrlMang::__state_led_calibration, this).detach(1024, 0, 4, -1);
                break;
            case DeviceState::RUNNING:
                // __state_led_running();
                utcollab::Task(&CtrlMang::__state_led_running, this).detach(1024, 0, 4, -1);
                break;
            case DeviceState::ERROR:
                // __state_led_error();
                utcollab::Task(&CtrlMang::__state_led_error, this).detach(1024, 0, 4, -1);
                break;
            default:
                break;
            }
        }
    }

private:
    /// @brief 电机的运动范围，单位为角度
    std::pair<float, float> __motor_motion_range{0.0f, 45.0f};

    /// @brief 归一化的目录位置，范围在0~1之间
    float __target_normalized_postion{0.0f};

    /// @brief 初始化状态的led显示
    void __state_led_init()
    {
        uint32_t step{1};
        while (device_state == DeviceState::INIT)
        {

            switch (step & 0x01)
            {
            case 0:
                state_led1.off();
                state_led2.off();
                state_led3.off();
                break;
            case 1:
                state_led1.off();
                state_led2.off();
                state_led3.on();
                step = 1;
            default:
                break;
            }
            utcollab::Task::sleep_for(500);
        }
    }

    /// @brief 配对状态的led显示
    void __state_led_pairing()
    {
        uint32_t step{0};
        while (device_state == DeviceState::PAIRING)
        {
            switch (step++ & 0x07)
            {
            case 0:
                state_led1.off();
                state_led2.off();
                state_led3.off();
                break;
            case 1:
                state_led1.on();
                state_led2.off();
                state_led3.off();
                break;
            case 2:
                state_led1.off();
                state_led2.on();
                state_led3.off();
                break;
            case 3:
                state_led1.off();
                state_led2.off();
                state_led3.on();
                step = 1;
                break;
            }
            utcollab::Task::sleep_for(350);
        }
    }

    /// @brief 电机校准状态的led显示
    void __state_led_calibration()
    {
        StateLED *state_led{&state_led1};
        switch (motor_func_mode)
        {
        case MotorFuncMode::FOCUS:
            state_led = &state_led1;
            break;
        case MotorFuncMode::ZOOM:
            state_led = &state_led2;
            break;
        case MotorFuncMode::IRIS:
            state_led = &state_led3;
            break;
        default:
            break;
        }
        while (device_state == DeviceState::CALIBRATION)
        {
            state_led->toggle();
            utcollab::Task::sleep_for(500);
        }
    }

    /// @brief 运行状态的led显示
    void __state_led_running()
    {
        while (device_state == DeviceState::RUNNING)
        {
            switch (motor_func_mode)
            {
            case MotorFuncMode::FOCUS:
                state_led1.on();
                state_led2.off();
                state_led3.off();
                break;
            case MotorFuncMode::ZOOM:
                state_led1.off();
                state_led2.on();
                state_led3.off();
                break;
            case MotorFuncMode::IRIS:
                state_led1.off();
                state_led2.off();
                state_led3.on();
                break;
            default:
                break;
            }
            utcollab::Task::sleep_for(500);
        }
    }

    /// @brief 错误状态的led显示
    void __state_led_error()
    {
        while (device_state == DeviceState::ERROR)
        {
            state_led1.on();
            state_led2.on();
            state_led3.on();
            utcollab::Task::sleep_for(500);
        }
    }

    /// @brief 更新按键事件
    void __update_key_loop()
    {
        UTINFO("Registered buttons:", Keyboard::instance().get_keys().size());

        utools::time::make_stable_interval_invoker(
            static_cast<void (Keyboard::*)(void)>(&Keyboard::update), &Keyboard::instance())
            .set_invoker_interval(10)
            .run_forever();
    }

    /// @brief 处理按键事件
    /// @param key_id 按键ID
    /// @param event 事件类型
    void __on_key_event(const int32_t key_id, const Keyboard::Event &event)
    {
        if (key_id == KEY_SHIFT)
        {
            DeviceState new_device_state = device_state;
            static uint8_t count_running = 0, count_calibration = 0, count_pairing = 0;
            switch (event)
            {
            case Keyboard::Event::LONG_PRESS:
                // 处理长按事件
                // UTDEBUG("Shift long press");
                break;
            case Keyboard::Event::PRESSED:
                // 处理按下事件
                UTDEBUG("Shift pressed");
                break;
            case Keyboard::Event::DOUBLE_CLICK:
                // TODO: 处理双击事件
                // UTDEBUG("Shift double click");
                break;
            case Keyboard::Event::RELEASED:
                // 处理释放事件
                if (device_state == DeviceState::RUNNING)
                {
                    if (count_running >= 3)
                    {
                        // 设备状态：RUNNING -> PAIRING (>3s)
                        new_device_state = DeviceState::PAIRING;
                    }
                    else if (count_running >= 1)
                    {
                        // 设备状态：RUNNING -> CALIBRATION (>1s)
                        new_device_state = DeviceState::CALIBRATION;
                    }
                }
                else if (device_state == DeviceState::CALIBRATION && count_calibration >= 1)
                {
                    // 设备状态：CALIBRATION -> RUNNING (>1s)
                    new_device_state = DeviceState::RUNNING;
                }
                else if (device_state == DeviceState::PAIRING && count_pairing >= 3)
                {
                    // 设备状态：PAIRING -> RUNNING (>3s)
                    new_device_state = DeviceState::RUNNING;
                }
                count_running = 0;
                count_calibration = 0;
                count_pairing = 0;
                UTDEBUG("Shift released");
                break;
            case Keyboard::Event::PERIOD_TRIGGER:
                // TODO: 处理周期触发事件
                // 记录长按时间，在释放时处理
                if (device_state == DeviceState::RUNNING)
                {
                    count_running++;
                }
                else if (device_state == DeviceState::CALIBRATION)
                {
                    count_calibration++;
                }
                else if (device_state == DeviceState::PAIRING)
                {
                    count_pairing++;
                }
                UTDEBUG("Shift period trigger");
                break;
            default:
                break;
            }
            set_device_state(new_device_state);
        }
    }
};

#endif // __CTRL_MANG_H__
