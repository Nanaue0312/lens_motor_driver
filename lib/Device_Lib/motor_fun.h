#ifndef __MOTOR_FUN__
#define __MOTOR_FUN__
#include <SimpleFOC.h>
#include "kth78xx.h"
#include <STM32FreeRTOS.h>
#include <atomic>

extern BLDCMotor motor;
extern float target_angle;
extern float target_angle_rec_uart;
extern float received_frm_data;
extern uint16_t received_slope_f;
extern int16_t dataToSave[4];
extern float current_angle;
// 最大角度
extern float max_angle;
// 最小角度
extern float min_angle;
void motor_work(float target_angle_rec, float slope);
void motor_init();
void TaskCalibration(void *pvParameters);
void serialReceiveUserCommand();
void TaskRecUart(void *pvParameters);

#endif // !__MOTOR_FUN__
