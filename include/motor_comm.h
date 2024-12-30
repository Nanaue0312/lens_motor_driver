#ifndef __MOTOR_COMM_H__
#define __MOTOR_COMM_H__
#include <Arduino.h>
#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct alignas(2) lens_motor_data_t
    {
        uint32_t mac;    // MAC地址
        uint32_t ts;     // 时间戳
        uint8_t funcode; // 功能码，1为聚焦电机(FOCUS)，2为缩放电机(ZOOM)，3为光圈电机(IRIS)
        float value;     // 0.0 ~ 1.0
        /// @note flag_***状态位
        uint8_t flag_status_cal : 2;   // 电机状态标志位，0未使用，1未使用，2校准中，3校准完成
        uint8_t flag_status_speed : 2; // 电机状态标志位，0未使用，1慢速，2中速，3快速
        uint8_t flag_reserved : 4;     // 保留
    } lens_motor_data_t;
#ifdef __cplusplus
}
#endif

#endif // !__MOTOR_COMM_H__
