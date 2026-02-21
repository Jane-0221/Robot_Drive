#ifndef __ARM_H__
#define __ARM_H__

#include "can_receive_send.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "crc_ccitt.h"

// ==================== 硬件资源统一定义 ====================
// CAN 句柄定义（两个 CAN 接口同等重要）
#define CAN_HANDLE_1          (&hfdcan1)
#define CAN_HANDLE_2          (&hfdcan2)

// 电机 ID 定义
#define MOTOR_LINGZU_1_ID       0x01    // 灵足电机 ID
#define MOTOR_LINGZU_2_ID       0x02    // 灵足电机 ID
#define MOTOR_LINGZU_3_ID       0x03    // 灵足电机 ID

#define MOTOR_DAMIAO_4_ID     0x04    // 达妙电机4 ID
#define MOTOR_DAMIAO_5_ID     0x05    // 达妙电机5 ID
#define MOTOR_DAMIAO_6_ID     0x06    // 达妙电机6 ID
#define MOTOR_YUSHU_1_ID      2       // 宇树电机1 ID
#define MOTOR_YUSHU_2_ID      3       // 宇树电机2 ID
// =========================================================

// 灵足电机对象
extern RobStride_Motor_t motor1;
// 灵足电机对象
extern RobStride_Motor_t motor2;
// 灵足电机对象
extern RobStride_Motor_t motor3;

// 达妙电机状态枚举
typedef enum Enum_Motor_DM_Status
{
    Motor_DM_Status_DISABLE = 0,
    Motor_DM_Status_ENABLE,
} Motor_DM_Status;
extern Motor_DM_Status DM_Status[6];

// 函数声明
void Arm_Init(void);
void Arm_motor1(void);
void Arm_motor2(void);
void Arm_motor3(void);
void Arm_motor4(void);
void Arm_motor5(void);
void Arm_motor6(void);
void Arm_all_tx(void);

#endif