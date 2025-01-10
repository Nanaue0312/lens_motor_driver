#ifndef __BROADCAST_DATA_H__
#define __BROADCAST_DATA_H__
#include "Arduino.h"
/// @brief 广播数据结构体
    typedef struct alignas(2) broadcast_data_t
    {
        uint32_t mac{0}; // MAC地址
        uint32_t ts{0};  // 时间戳
        /// @note 控制参数
        float focus;   // 聚焦值
        float zoom;    // 缩放值
        float iris;    // 光圈值
        float hitwall; // 撞墙点聚焦值
        /// @note ***flag_***标志位
        uint16_t motorflag_focus_status : 2 {2}; // focus电机标志位，0校准，1校准中，2运行中（校准完成），3停止工作
        uint16_t motorflag_focus_speed : 2 {2};  // focus电机速度，0未使用，1慢速，2中速，3快速
        uint16_t motorflag_zoom_status : 2 {2};  // zoom电机标志位，0校准，1校准中，2运行中（校准完成），3停止工作
        uint16_t motorflag_zoom_speed : 2 {2};   // zoom电机保留，0未使用，1慢速，2中速，3快速
        uint16_t motorflag_iris_status : 2 {2};  // iris电机标志位，0校准，1校准中，2运行中（校准完成），3停止工作
        uint16_t motorflag_iris_speed : 2 {2};   // iris电机保留，0未使用，1慢速，2中速，3快速
        /// @note ueflag_***标志位
        uint16_t ueflag_recording : 1 {0};  // 录像标志位
        uint16_t ueflag_focal_plane : 1 {0}; // 是否在焦平面
        uint16_t ueflag_hit_wall : 1 {0};    // 是否锁定
        /// @note flag_***状态位
        uint16_t flag_reserved : 1;        // 保留
    } broadcast_data_t;
#endif // __BROADCAST_DATA_H__
