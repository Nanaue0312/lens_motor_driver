/// @brief 功能定义，不要修改

#ifndef __DEVICE_TYPE_DEF_H__
#define __DEVICE_TYPE_DEF_H__

#include <cstdint>

enum class DeviceType : uint8_t
{
    NOTUSED = 0,         // 未使用的节点
    UNKNOWN = 1,         // 未知节点
    ROCKER_WHEEL = 2,    // 摇轮控制器
    ROCKER_ARM = 3,      // 摇臂控制器
    FOLLOW_FOCUS = 4,    // 跟焦器
    PURPOSE_TRIGGER = 5, // 通用触发器
    VIRTUAL_BATON = 6,   // 虚拟指挥棒
    TREADMILLS = 7,      // 跑步机
    STAIRCASE_LIFT = 8,  // 楼梯机
    LOGIC_SLIDER = 9,    // Logic 推杆
    RAZER_HANDLE = 10,   // 雷蛇手柄
    XBOX_HANDLE = 11     // Xbox手柄
};

#endif // __DEVICE_TYPE_DEF_H__