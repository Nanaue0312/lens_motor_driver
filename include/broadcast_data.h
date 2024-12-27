#ifndef __BROADCAST_DATA_H__
#define __BROADCAST_DATA_H__
#include "Arduino.h"
/// @brief 广播数据结构体
typedef struct alignas(2) broadcast_data_t
{
    uint32_t mac; // MAC地址
    uint32_t ts;  // 时间戳
    /// @note 控制参数
    float focus; // 聚焦值
    float zoom;  // 缩放值
    float iris;  // 光圈值
    /// @note motorflag_***标志位
    uint16_t motorflag_focus : 3;    // focus电机标志位，0不在线，1校准，2慢速，3中速，4快速
    uint16_t motorflag_zoom : 3;     // zoom电机标志位，0不在线，1校准，2慢速，3中速，4快速
    uint16_t motorflag_iris : 3;     // iris电机标志位，0不在线，1校准，2慢速，3中速，4快速
    uint16_t motorflag_reserved : 7; // 保留
    /// @note ueflag_***标志位
    uint8_t ueflag_recording : 1;   // 录像标志位
    uint8_t ueflag_focal_plane : 1; // 是否在焦平面
    uint8_t ueflag_hit_wall : 1;    // 是否锁定
    uint8_t ueflag_reserved : 5;    // 保留
    /// @note radioflag_***通讯通道控制标志位
    uint8_t radioflags_motor_func : 4; // 电机功能，1为Focus，2为zoom电机，3为iris电机，4为virtual lens控制
    uint8_t radioflag_reserved : 4;    // 保留
} broadcast_data_t;
#endif // __BROADCAST_DATA_H__
