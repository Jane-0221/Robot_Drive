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
// 遥控器值
#define LOW_VALUE 353
#define MID_VALUE 1024
#define HIGH_VALUE 1694
#define RANGE 50
// 升降继电器电平定义：低电平触发吸合，高电平断开控制引脚
#define RELAY1_PIN GPIO_PIN_0
#define RELAY1_PORT GPIOA
#define RELAY2_PIN GPIO_PIN_2
#define RELAY2_PORT GPIOA
//
#define RELAY_ON GPIO_PIN_RESET // 继电器吸合（低电平）
#define RELAY_OFF GPIO_PIN_SET  // 继电器断开（高电平）

BOOM_ARM_Stats boom_arm_status; // 机械臂气泵状态
LIFT_State lift_state;          // 伸降杆状态





void Head_Motor_Control_Updata(void)
{
if(SBUS_CH.CH6==HIGH_VALUE)
{
    Daran_motor_data[0].target_angle=0;
}
else if(SBUS_CH.CH6==MID_VALUE)
{
    Daran_motor_data[0].target_angle=90;
}
else if(SBUS_CH.CH6==LOW_VALUE)
{
    Daran_motor_data[0].target_angle=180;
}

}

/**
 * @brief  控制电机正转
 * @note   IN1(PA0)=低电平（吸合）、IN2(PA1)=高电平（断开）
 *         电机A端接24V+，B端接24V-，正向转动
 * @param  无
 * @retval 无
 */
void Motor_Forward(void)
{
    // 第一路继电器吸合（低电平）
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_ON);
    // 第二路继电器断开（高电平）
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_OFF);
}

/**
 * @brief  控制电机反转
 * @note   IN1(PA0)=高电平（断开）、IN2(PA1)=低电平（吸合）
 *         电机A端接24V-，B端接24V+，反向转动
 * @param  无
 * @retval 无
 */
void Motor_Reverse(void)
{
    // 第一路继电器断开（高电平）
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_OFF);
    // 第二路继电器吸合（低电平）
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_ON);
}

/**
 * @brief  控制电机停止
 * @note   IN1和IN2都为高电平，两路继电器都断开，电机两端悬空，停止转动
 * @param  无
 * @retval 无
 */
void Motor_Stop(void)
{
    // 两路继电器都断开（高电平）
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_OFF);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_OFF);
}
void Up_Down_Motor_Control_Updata(void)
{
    switch (lift_state)
    {
    case 0:
        Motor_Forward();
        break;
    case 1:
        Motor_Reverse();
        break;
    case 2:
    default: // 兜底，确保任何情况都有处理
        Motor_Stop();
        break;
    }
}
void Up_Down_Motor_Control(void)
{

    switch (SBUS_CH.CH7)
    {
    case HIGH_VALUE:
        lift_state = LIFT_UP;
        break;
    case LOW_VALUE:
        lift_state = LIFT_DOWN;
         break;
    case MID_VALUE:
    default: // 兜底，确保任何情况都有处理
        lift_state = LIFT_STOP;
        break;
    }
}

void remote_control_init()
{
    GPIO_init();

    boom_arm_status = ARM_BOOM_OFF; // 机械臂
}

void GPIO_init() // 电子开关初始化
{
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET); // PD14//气泵1

    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET); // PD15//电磁阀2

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); // PB8

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET); // PB9

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_14, GPIO_PIN_RESET); // PE14

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_0, GPIO_PIN_RESET); // PE0，左储矿

    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_1, GPIO_PIN_RESET); // PE1

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_10, GPIO_PIN_RESET); // PC10 气泵

    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_11, GPIO_PIN_RESET); // PC11，右储矿
}
