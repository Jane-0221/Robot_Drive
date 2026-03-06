#include "remote_control.h"
#include "ramp_generator.h"
#include "IMU_updata.h"
#include "Stm32_time.h"
#include <tim.h>
#include "pid.h"
#include "UART_data_txrx.h"
#include "Sbus.h"
#include "head.h"
#include "arm.h"
#include "lift_control.h"
#include "pump_control.h"
#include "arm_sv.h"
#include "uart_protocol.h" // 添加PC通信协议头文件

// 遥控器值
#define LOW_VALUE 353
#define MID_VALUE 1024
#define HIGH_VALUE 1694
#define RANGE 50

void remote_control_init()
{
}

void Pump_Control_Updata(void)
{
    if (SBUS_CH.CH8 == HIGH_VALUE)
    {
        pump_state = PUMP_ON;
    }
    else if (SBUS_CH.CH8 == LOW_VALUE)
    {
        pump_state = PUMP_OFF;
    }
}

void Head_Motor_Control_Updata(void)
{
    if (SBUS_CH.CH6 == HIGH_VALUE)
    {
        Daran_motor_data[0].target_angle = 0;
        Daran_motor_data[1].target_angle = 0;

        duties_tx.duty0 = 0.125;
        duties_tx.duty1 = 0.125;
        duties_tx.duty2 = 0.125;
        duties_tx.duty3 = 0.125;
        duties_tx.duty4 = 0.125;
        duties_tx.duty5 = 0.125;
    }
    else if (SBUS_CH.CH6 == MID_VALUE)
    {
        Daran_motor_data[0].target_angle = 90;
        Daran_motor_data[1].target_angle = 90;

        duties_tx.duty0 = 0.075;
        duties_tx.duty1 = 0.075;
        duties_tx.duty2 = 0.075;
        duties_tx.duty3 = 0.075;
        duties_tx.duty4 = 0.075;
        duties_tx.duty5 = 0.075;
    }
    else if (SBUS_CH.CH6 == LOW_VALUE)
    {
        Daran_motor_data[0].target_angle = 180;
        Daran_motor_data[1].target_angle = 180;

        duties_tx.duty0 = 0.025;
        duties_tx.duty1 = 0.025;
        duties_tx.duty2 = 0.025;
        duties_tx.duty3 = 0.025;
        duties_tx.duty4 = 0.025;
        duties_tx.duty5 = 0.025;
    }
}

void Up_Down_Motor_Control_Updata(void)
{
    switch (SBUS_CH.CH7)
    {
    case HIGH_VALUE:
        aim_tx_height = 100;
        break;
    case LOW_VALUE:
        aim_tx_height = 700;
        break;
    case MID_VALUE:
        aim_tx_height = 400;
        break;
    default:
        break;
    }
}

// PC控制函数实现

/**
 * @brief PC控制气泵更新函数
 * @note 使用PC传入的气泵状态信息进行控制
 */
void PC_Pump_Control_Updata(void)
{
    // 使用PC传入的气泵状态信息
    if (pc_dn_data.pc_pump_state == 1)
    {
        pump_state = PUMP_ON;
    }
    else if (pc_dn_data.pc_pump_state == 0)
    {
        pump_state = PUMP_OFF;
    }
}

/**
 * @brief PC控制头部电机更新函数
 * @note 使用PC传入的头部电机角度信息进行控制
 */
void PC_Head_Motor_Control_Updata(void)
{
    // 使用PC传入的头部电机角度信息（索引0和1对应头部电机）
    // pc_target_angles[0] 和 pc_target_angles[1] 是0.1度的单位，需要转换为度
    Daran_motor_data[0].target_angle = (float)pc_dn_data.pc_target_angles[0] / 10.0f;
    Daran_motor_data[1].target_angle = (float)pc_dn_data.pc_target_angles[1] / 10.0f;

    // 根据角度设置占空比（示例逻辑，可根据实际需求调整）
    float duty_base = 0.025f + (Daran_motor_data[0].target_angle / 180.0f) * 0.1f;
    
    duties_tx.duty0 = duty_base;
    duties_tx.duty1 = duty_base;
    duties_tx.duty2 = duty_base;
    duties_tx.duty3 = duty_base;
    duties_tx.duty4 = duty_base;
    duties_tx.duty5 = duty_base;
}

/**
 * @brief PC控制升降电机更新函数
 * @note 使用PC传入的升降目标高度信息进行控制
 */
void PC_Up_Down_Motor_Control_Updata(void)
{
    // 使用PC传入的升降目标高度信息（0.1mm单位）
    aim_tx_height = pc_dn_data.pc_target_lift_height;
}