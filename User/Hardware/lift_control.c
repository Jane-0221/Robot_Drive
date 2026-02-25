#include "lift_control.h"
#include "remote_control.h"
#include "usart.h"
#include "UART_data_txrx.h"
#include "stp23l.h"
// 定义全局变量
LIFT_State lift_state = LIFT_STOP;      // 初始为停止
uint16_t lift_current_height = 0;       // 初始高度0

//定义局部变量
int16_t lift_height_final = 0; // 升降杆最终高度值（每帧更新一次）
// 底层电机控制函数（内部使用）
static void Motor_Forward(void);
static void Motor_Reverse(void);
static void Motor_Stop(void);

/**
 * @brief  初始化升降杆控制
 * @note   确保电机停止，状态复位；引脚初始化通常在硬件初始化中完成
 * @param  无
 * @retval 无
 */
void Lift_Init(void)
{
    Motor_Stop();
    lift_state = LIFT_STOP;
    lift_current_height = 0;
}

/**
 * @brief  设置上升状态
 * @param  无
 * @retval 无
 */
void Lift_Up(void)
{
    lift_state = LIFT_UP;
}

/**
 * @brief  设置下降状态
 * @param  无
 * @retval 无
 */
void Lift_Down(void)
{
    lift_state = LIFT_DOWN;
}

/**
 * @brief  设置停止状态
 * @param  无
 * @retval 无
 */
void Lift_Stop(void)
{
    lift_state = LIFT_STOP;
}

/**
 * @brief  设置指定状态
 * @param  state 要设置的状态
 * @retval 无
 */
void Lift_SetState(LIFT_State state)
{
    lift_state = state;
}

/**
 * @brief  获取当前状态
 * @retval 当前状态
 */
LIFT_State Lift_GetState(void)
{
    return lift_state;
}

/**
 * @brief  根据当前状态更新电机控制
 * @note   此函数需周期性调用（例如每10ms），以执行实际控制
 * @param  无
 * @retval 无
 */
void Lift_UpdateMotor(void)
{
    switch (lift_state)
    {
    case LIFT_UP:
        Motor_Forward();
        break;
    case LIFT_DOWN:
        Motor_Reverse();
        break;
    case LIFT_STOP:
    default:
        Motor_Stop();
        break;
    }
}

/**
 * @brief  刷新高度值（从传感器读取）
 * @note   用户需根据实际硬件在此添加读取代码，例如ADC或编码器
 * @param  无
 * @retval 无
 */
void Lift_RefreshHeight(void)
{

            if(stp23l_data.parse_ok == 1)
        {
            lift_height_final = STP23L_GetFinalDistPerFrame(); // 核心调用
            STP23L_ClearOkFlag(); // 清除标志，准备下一帧解析
        }
    // 示例：假设高度由ADC采集，通道1
    // lift_current_height = HAL_ADC_GetValue(&hadc1);
    // 此处留空，请根据实际硬件实现
}

/**
 * @brief  获取当前高度
 * @retval 当前高度值
 */
uint16_t Lift_GetHeight(void)
{
    return lift_current_height;
}

/* **************** 底层电机控制（与您提供的代码一致） **************** */

/**
 * @brief  电机正转（上升）
 * @note   IN1(PA0)=低电平吸合，IN2(PA2)=高电平断开 → 电机正转
 * @param  无
 * @retval 无
 */
static void Motor_Forward(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_ON);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_OFF);
}

/**
 * @brief  电机反转（下降）
 * @note   IN1(PA0)=高电平断开，IN2(PA2)=低电平吸合 → 电机反转
 * @param  无
 * @retval 无
 */
static void Motor_Reverse(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_OFF);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_ON);
}

/**
 * @brief  电机停止
 * @note   两路继电器均断开（高电平），电机两端悬空
 * @param  无
 * @retval 无
 */
static void Motor_Stop(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_OFF);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_OFF);
}