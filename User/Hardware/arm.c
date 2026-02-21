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


// 宇树电机数据结构
MotorData_t YS_8010_data[2] = {0};
MotorCmd_t YS_8010_cmd[2] = {0};
HAL_StatusTypeDef tx_res;
HAL_StatusTypeDef rx_res;

// 达妙电机状态
extern Motor_DM_Status DM_Status[6];
RobStride_Motor_t motor1; // 灵足电机对象
RobStride_Motor_t motor3; // 灵足电机对象
void Arm_Init()
{
    /* 灵足电机初始化（使用 CAN2）*/
    //1号电机初始化
    RobStride_Motor_Init(&motor1, MOTOR_LINGZU_1_ID, false);
    Get_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(&motor1, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor1, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
   RobStride_Motor_ProactiveEscalationSet(&motor1,CAN_HANDLE_2,0x01);//00为关闭主动上报，01为开启主动上报

       RobStride_Motor_Init(&motor3, MOTOR_LINGZU_3_ID, false);
    Get_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005);
    HAL_Delay(10);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7005, CSP_control_mode, 'j');
    Enable_Motor(&motor3, (hcan_t *)CAN_HANDLE_2);
    Set_RobStride_Motor_parameter(&motor3, CAN_HANDLE_2, 0X7017, 1.0f, 'p');
    HAL_Delay(10);
   RobStride_Motor_ProactiveEscalationSet(&motor3,CAN_HANDLE_2,0x01);//00为关闭主动上报，01为开启主动上报

   /* LZ电机初始化（使用 CAN2）*/
    //2号电机初始化
    lz_enable_motor(2, 2); // 使能ID为2的电机
    HAL_Delay(10);
    lz_set_mode(2, 2, LZ_MODE_POSITION); // 设置为位置控制模式
    HAL_Delay(10);




    /* 宇树电机初始化（使用 UART，与 CAN 无关）*/
    YS_8010_cmd[0].id = MOTOR_YUSHU_1_ID;
    YS_8010_cmd[0].mode = 1;
    YS_8010_cmd[0].K_P = 0.02f;
    YS_8010_cmd[0].K_W = 0.0f;
    YS_8010_cmd[0].Pos = 100.0f;
    YS_8010_cmd[0].W = 0.0f;
    YS_8010_cmd[0].T = 0.0f;

    YS_8010_cmd[1].id = MOTOR_YUSHU_2_ID;
    YS_8010_cmd[1].mode = 1;
    YS_8010_cmd[1].K_P = 0.02f;
    YS_8010_cmd[1].K_W = 0.0f;
    YS_8010_cmd[1].Pos = 100.0f;
    YS_8010_cmd[1].W = 0.0f;
    YS_8010_cmd[1].T = 0.0f;

    /* 达妙电机初始化（使用 CAN2）*/
    arm_motor_init(&arm_motor[Motor4], MOTOR_DAMIAO_4_ID, POS_MODE);
    arm_motor_init(&arm_motor[Motor5], MOTOR_DAMIAO_5_ID, POS_MODE);
    arm_motor_init(&arm_motor[Motor6], MOTOR_DAMIAO_6_ID, POS_MODE);

    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, POS_MODE);
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, POS_MODE);
    enable_motor_mode(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, POS_MODE);

    vel_motor.MT04 = 5.0;
    pos_motor.MT04 = 0;
}

void Arm_motor1()
{
    // RobStride_Motor_ProactiveEscalationSet(&motor1,CAN_HANDLE_2,0x01);
    RobStride_Motor_CSP_control(&motor1, CAN_HANDLE_2, 20.0f, 10.0f);
}

void Arm_motor2()
{
    //yushu
    // modify_data(&YS_8010_cmd[0]);
    // HAL_UART_Transmit(&huart3, (uint8_t *)&YS_8010_cmd[0].motor_send_data, sizeof(YS_8010_cmd[0].motor_send_data), 1);
    // HAL_UART_Receive(&huart3, (uint8_t *)&YS_8010_data[0].motor_recv_data, sizeof(YS_8010_data[0].motor_recv_data), 1);
   // LZ电机位置控制（ID=2）
    // 使用CAN2总线，目标位置设为0.0弧度，速度设为1.0弧度/秒
    // 可根据实际需求调整目标位置和速度参数
    lz_set_position(2, 2, 0.0f, 1.0f);



}

void Arm_motor3()
{
    //yushu
    // modify_data(&YS_8010_cmd[1]);
    // HAL_UART_Transmit(&huart3, (uint8_t *)&YS_8010_cmd[1].motor_send_data, sizeof(YS_8010_cmd[1].motor_send_data), 1);
    // HAL_UART_Receive(&huart3, (uint8_t *)&YS_8010_data[1].motor_recv_data, sizeof(YS_8010_data[1].motor_recv_data), 1);
     RobStride_Motor_CSP_control(&motor3, CAN_HANDLE_2, 1.0f, 10.0f);



}

void Arm_motor4()
{
    set_DM_mode(Motor4, POS_MODE);
    set_DM_pos_vel(pos_motor.MT04, vel_motor.MT04, Motor4);
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_4_ID, 5, 10);
    pos_motor.MT04 = 20;
    Pre_Flag_damiao[4] = Flag_damiao[4];
}

void Arm_motor5()
{
    set_DM_mode(Motor5, POS_MODE);
    set_DM_pos_vel(pos_motor.MT05, vel_motor.MT05, Motor5);
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_5_ID, 10, 1);
    Pre_Flag_damiao[5] = Flag_damiao[5];
}

void Arm_motor6()
{
    set_DM_mode(Motor6, POS_MODE);
    set_DM_pos_vel(pos_motor.MT06, vel_motor.MT06, Motor6);
    pos_motor.MT06 = 10;
    pos_speed_ctrl(CAN_HANDLE_2, MOTOR_DAMIAO_6_ID, arm_motor[Motor6].ctrl.pos_set, 1);
    Pre_Flag_damiao[6] = Flag_damiao[6];
}
void Arm_all_tx()
{
    // Arm_motor1();
    // osDelay(1);
    // Arm_motor2();
    // osDelay(1);
    Arm_motor3();
    osDelay(1);
    // Arm_motor4();
    // osDelay(1);
    //  Arm_motor5();
    //  osDelay(1);
    // Arm_motor6();
    //  osDelay(1);


}