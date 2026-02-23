#ifndef __HEAD_H__
#define __HEAD_H__
#include "can_receive_send.h"
#include "ktech_motor.h"
// CAN 句柄定义
#define CAN_HANDLE_1 (&hfdcan1)
#define MOTOR_LINKONG_1_ID 0x01 // 电机1 ID
#define MOTOR_LINKONG_2_ID 0x02 // 电机2 ID
#define DIR_CW                   0x00    // 顺时针
#define DIR_CCW                  0x01    // 逆时针
typedef struct
{
    float current_velocity; // 当前速度（单位：rad/s）
    float current_angle;    // 当前角度（单位：rad ）
    uint8_t direction;      // 0:顺时针, 1:逆时针
    uint32_t target_angle;  // 目标角度，18000 对应 180° (0.01°/LSB)
    uint16_t max_speed;     // 最大转速 360 dps (即 360°/秒)
} Head_MotorData_t;

extern Head_MotorData_t head_motor_data[2];

// 电机反馈帧结构体定义
extern KTech_Motor_t motor_linkong[2];
extern void Head_Init(void);
extern void Head_Lk_motor1(void);
extern void Head_all_tx(void);


extern void Arm_Lk_Data_update(void);
#endif
