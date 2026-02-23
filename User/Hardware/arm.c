#include "arm.h"
#include "dm4310_drv.h"
#include "Robstride04.h"
#include "pid.h"
#include "remote_control.h"
#include "gpio.h"
#include "gom_protocol.h"
#include "usart.h"
#include "LZ_motor_driver.h"
#include "FreeRTOS.h"
#include "task.h"
#include "DrEmpower_can.h"
ShoulderType_t g_ShoulderType; // 肩部类型

struct servo_state servo_state_daran[3];
struct servo_volcur servo_volcur_daran[3];
// 达妙电机状态
extern Motor_DM_Status DM_Status[6];
RobStride_Motor_t motor1; // 灵足电机对象
RobStride_Motor_t motor2; // 灵足电机对象
RobStride_Motor_t motor3; // 灵足电机对象
// 灵足肩电机数据
ArmMotorData_t Linzu_motor_data[3];
// 大然肩电机数据
ArmMotorData_t Daran_motor_data[3];
// 达妙电机数据
ArmMotorData_t Damiao_motor_data[3];

float reply_enable = 0.0f;
void Arm_Init()
{
    // 选择默认肩部类型
    // g_ShoulderType = SHOULDER_TYPE_LINGZU;
    g_ShoulderType = SHOULDER_TYPE_DARAN;
    /* 灵足电机初始化（使用 CAN2）*/
    // 1号电机初始化
    RobStride_Motor_Init(&motor1, MOTOR_LINGZU_1_ID, false);
    Get_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(&motor1, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor1, CAN_HANDLE_2, 0x01); // 00为关闭主动上报，01为开启主动上报
    // 2号电机初始化
    RobStride_Motor_Init(&motor2, MOTOR_LINGZU_2_ID, false);
    Get_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(&motor2, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor2, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor2, CAN_HANDLE_2, 0x01); // 00为关闭主动上报，01为开启主动上报
    // 3号电机初始化
    RobStride_Motor_Init(&motor3, MOTOR_LINGZU_3_ID, false);
    Get_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(&motor3, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
    RobStride_Motor_ProactiveEscalationSet(&motor3, CAN_HANDLE_2, 0x01); // 00为关闭主动上报，01为开启主动上报
                                                                         /* 大然电机初始化（使用 CAN2）*/
                                                                         // 1号电机初始化
                                                                         // clear_error(CAN_HANDLE_2, MOTOR_DARAN_1_ID);

    /* 大然电机初始化（使用 CAN2）*/
    clear_error(CAN_HANDLE_2, MOTOR_DARAN_1_ID);
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_1_ID, 2);
    clear_error(CAN_HANDLE_2, MOTOR_DARAN_2_ID);
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_2_ID, 2);
    clear_error(CAN_HANDLE_2, MOTOR_DARAN_3_ID);
    set_mode(CAN_HANDLE_2, MOTOR_DARAN_3_ID, 2);

    /* 达妙电机初始化（使用 CAN2）*/
    arm_motor_init(&arm_motor[Motor4], MOTOR_DAMIAO_4_ID, POS_MODE);
    arm_motor_init(&arm_motor[Motor5], MOTOR_DAMIAO_5_ID, POS_MODE);
    arm_motor_init(&arm_motor[Motor6], MOTOR_DAMIAO_6_ID, POS_MODE);

    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE);
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE);
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE);
    // 灵足肩电机数据
    Linzu_motor_data[0].target_angle = 10.0f;
    Linzu_motor_data[1].target_angle = 10.0f;
    Linzu_motor_data[2].target_angle = 10.0f;
    Linzu_motor_data[0].target_velocity = 1.0f;
    Linzu_motor_data[1].target_velocity = 1.0f;
    Linzu_motor_data[2].target_velocity = 1.0f;
    // 大然肩电机数据
    Daran_motor_data[0].target_angle = 190.0f;
    Daran_motor_data[1].target_angle = 10.0f;
    Daran_motor_data[2].target_angle = 10.0f;

    Daran_motor_data[0].target_velocity = 20.0f;
    Daran_motor_data[1].target_velocity = 20.0f;
    Daran_motor_data[2].target_velocity = 20.0f;
}

void Arm_Linzu_motor1()
{
    RobStride_Motor_CSP_control(&motor1, CAN_HANDLE_2, Linzu_motor_data[0].target_angle, Linzu_motor_data[0].target_velocity);
}

void Arm_Linzu_motor2()
{
    RobStride_Motor_CSP_control(&motor2, CAN_HANDLE_2, Linzu_motor_data[1].target_angle, Linzu_motor_data[1].target_velocity);
}

void Arm_Linzu_motor3()
{

    RobStride_Motor_CSP_control(&motor3, CAN_HANDLE_2, Linzu_motor_data[2].target_angle, Linzu_motor_data[2].target_velocity);
}

void Arm_Daran_motor1()
{
    // servo_state_daran[0] = get_state(CAN_HANDLE_2, MOTOR_DARAN_1_ID);
    servo_volcur_daran[0] = get_volcur(CAN_HANDLE_2, MOTOR_DARAN_1_ID);
    set_angle(CAN_HANDLE_2, MOTOR_DARAN_1_ID, Daran_motor_data[0].target_angle, Daran_motor_data[0].target_velocity, 10.0f, 1);
}
void Arm_Daran_motor2()
{
    // get_volcur(CAN_HANDLE_2, MOTOR_DARAN_2_ID);
    // servo_state_daran[1] = get_state(CAN_HANDLE_2, MOTOR_DARAN_2_ID);
    // set_angle(CAN_HANDLE_2, MOTOR_DARAN_2_ID, Daran_motor_data[1].target_angle, Daran_motor_data[1].target_velocity, 10.0f, 1);
}

void Arm_Daran_motor3()
{
    set_angle(CAN_HANDLE_2, MOTOR_DARAN_3_ID, Daran_motor_data[2].target_angle, Daran_motor_data[2].target_velocity, 10.0f, 1);
}

void Arm_Damiao_motor4()
{
    set_DM_mode(Motor4, POS_MODE);
    set_DM_pos_vel(pos_motor.MT04, vel_motor.MT04, Motor4);
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, 5, 10);
    pos_motor.MT04 = 20;
}

void Arm_Damiao_motor5()
{
    set_DM_mode(Motor5, POS_MODE);
    set_DM_pos_vel(pos_motor.MT05, vel_motor.MT05, Motor5);
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, 10, 1);
}

void Arm_Damiao_motor6()
{
    set_DM_mode(Motor6, POS_MODE);
    set_DM_pos_vel(pos_motor.MT06, vel_motor.MT06, Motor6);
    pos_motor.MT06 = 10;
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, arm_motor[Motor6].ctrl.pos_set, 1);
}

void Arm_Linzu_Data_update()
{
    Linzu_motor_data[0].current_angle = motor1.Pos_Info.Angle;
    Linzu_motor_data[1].current_angle = motor2.Pos_Info.Angle;
    Linzu_motor_data[2].current_angle = motor3.Pos_Info.Angle;
    Linzu_motor_data[0].current_velocity = motor1.Pos_Info.Speed;
    Linzu_motor_data[1].current_velocity = motor2.Pos_Info.Speed;
    Linzu_motor_data[2].current_velocity = motor3.Pos_Info.Speed;
}
// 所有电机数据更新
void Arm_All_Data_update()
{
    Arm_Linzu_Data_update();
}

void Arm_all_tx()
{

    if (g_ShoulderType == SHOULDER_TYPE_LINGZU)
    {
        Arm_Linzu_motor1();
        osDelay(1);
        Arm_Linzu_motor2();
        osDelay(1);
        Arm_Linzu_motor3();
        osDelay(1);
    }
    else if (g_ShoulderType == SHOULDER_TYPE_DARAN)
    {
        Arm_Daran_motor1();
        osDelay(1);
        Arm_Daran_motor2();
        osDelay(1);
        Arm_Daran_motor3();
        osDelay(1);
    }

    // Arm_Damiao_motor4();
    // osDelay(1);
    // Arm_Damiao_motor5();
    // osDelay(1);
    // Arm_Damiao_motor6();
    // osDelay(1);
}