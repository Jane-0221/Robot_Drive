#ifndef __ARM_H__
#define __ARM_H__

#include "can_receive_send.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "crc_ccitt.h"

// ==================== 硬件资源统一定义 ====================
// CAN 句柄定义（两个 CAN 接口同等重要）
#define CAN_HANDLE_1 (&hfdcan1)
#define CAN_HANDLE_2 (&hfdcan2)

// 电机 ID 定义
#define MOTOR_LINGZU_1_ID 0x01 // 灵足电机1 ID
#define MOTOR_LINGZU_2_ID 0x02 // 灵足电机2 ID
#define MOTOR_LINGZU_3_ID 0x03 // 灵足电机3 ID

#define MOTOR_DARAN_1_ID 0x0b // 大然电机11 ID
#define MOTOR_DARAN_2_ID 0x0c // 大然电机12 ID
#define MOTOR_DARAN_3_ID 0x0d // 大然电机13 ID

#define MOTOR_DAMIAO_4_ID 0x04 // 达妙电机4 ID
#define MOTOR_DAMIAO_5_ID 0x05 // 达妙电机5 ID
#define MOTOR_DAMIAO_6_ID 0x06 // 达妙电机6 ID
#define MOTOR_YUSHU_1_ID 2     // 宇树电机1 ID
#define MOTOR_YUSHU_2_ID 3     // 宇树电机2 ID
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

// 肩部类型选择枚举
typedef enum {
    SHOULDER_TYPE_LINGZU = 0,  // 灵足肩
    SHOULDER_TYPE_DARAN = 1    // 大然肩
} ShoulderType_t;
// 全局肩部选择变量
extern ShoulderType_t g_ShoulderType;
// 电机数据结构体
typedef struct {
    float current_velocity;   // 当前速度（单位：rad/s）
    float current_angle;      // 当前角度（单位：rad ）
    float target_angle;       // 目标角度（单位：rad ）
    float target_velocity;    // 目标速度（单位：rad/s ）
} ArmMotorData_t;
// 电机数据数组
// 全局肩部选择变量
//灵足肩电机数据
extern ArmMotorData_t Linzu_motor_data[3];
//大然肩电机数据
extern ArmMotorData_t Daran_motor_data[3];
//达妙电机数据
extern ArmMotorData_t Damiao_motor_data[3];
// 函数声明
void Arm_Init(void);
//灵足肩
void Arm_Linzu_motor1(void);
void Arm_Linzu_motor2(void);
void Arm_Linzu_motor3(void);
//大然肩
void Arm_Daran_motor1(void);
void Arm_Daran_motor2(void);
void Arm_Daran_motor3(void);
//达妙小臂
void Arm_Damiao_motor4(void);
void Arm_Damiao_motor5(void);
void Arm_Damiao_motor6(void);
//电机角度更新
void Arm_Linzu_Data_update(void);
//所有电机数据更新
extern void Arm_All_Data_update(void);
//发送所有电机数据
extern void Arm_all_tx(void);


#endif