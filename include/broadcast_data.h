#ifndef __BROADCAST_DATA_H__
#define __BROADCAST_DATA_H__
#include "Arduino.h"
/// @brief 广播数据结构体
typedef struct alignas(2) broadcast_data_t
{
    uint32_t mac;      // MAC地址
    uint32_t timecode; // 时间戳
    /// @note 控制参数
    float focus; // 聚焦值
    float zoom;  // 缩放值
    float iris;  // 光圈值
    /// @note ueflag_***标志位
    uint8_t ueflag_recording : 1;   // 录像标志位
    uint8_t ueflag_focal_plane : 1; // 是否在焦平面
    uint8_t ueflag_hit_wall : 1;    // 是否锁定
    uint8_t ueflag_reserved : 5;    // 保留
    /// @note motorflag_***标志位
    uint8_t motorflag_focus : 2;    // focus电机标志位
    uint8_t motorflag_zoom : 2;     // zoom电机标志位
    uint8_t motorflag_iris : 2;     // iris电机标志位
    uint8_t motorflag_reserved : 2; // 保留
    /// @note radioflag_***通讯通道控制标志位
    uint16_t radioflag_focus_motor : 1;   // focuse电机控制
    uint16_t radioflag_zoom_motor : 1;    // zoom电机控制
    uint16_t radioflag_iris_motor : 1;    // iris电机控制
    uint16_t radioflag_virtual_motor : 1; // 虚拟电机控制
    uint16_t radioflag_reserved : 12;     // 保留
} broadcast_data_t;
#endif // __BROADCAST_DATA_H__
