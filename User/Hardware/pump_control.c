#include "pump_control.h"
#include "gpio.h"
#include "stm32h7xx_hal.h"
#include "pt_sensor.h"
// 定义全局变量
PUMP_State pump_state = PUMP_OFF; // 初始为关闭
LIQUID_State liquid_state = LIQUID_NOT_SUCKED; // 初始为未吸到药品
// 压力阈值定义
#define PRESSURE_THRESHOLD_LOW  10.0f  // 压力低阈值
#define PRESSURE_THRESHOLD_HIGH 70.0f  // 压力高阈值
/**
 * @brief  初始化气泵控制
 * @note   确保气泵关闭，引脚初始化通常在硬件初始化中完成
 * @param  无
 * @retval 无
 */
void Pump_Init(void)
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
                                                           // 关闭气泵
    pump_state = PUMP_OFF;
}

/**
 * @brief  开启气泵
 * @note   直接设置状态并立即控制继电器
 * @param  无
 * @retval 无
 */

/**
 * @brief  根据当前状态更新继电器
 * @note   此函数需周期性调用（例如每10ms），以实现状态与硬件的同步。
 *        若使用了 Pump_On/Off 直接操作，则此函数可省略；但为统一管理，建议保留。
 * @param  无
 * @retval 无
 */
void Pump_Update(void)
{
    switch (pump_state)
    {
    case PUMP_ON:
        HAL_GPIO_WritePin(PUMP_RELAY_PORT, PUMP_RELAY_PIN, RELAY_OFF);
        HAL_GPIO_WritePin(SOLENOID_VALVE_PORT, SOLENOID_VALVE_PIN, RELAY_OFF);
        break;
    case PUMP_OFF:
    default:
        HAL_GPIO_WritePin(PUMP_RELAY_PORT, PUMP_RELAY_PIN, RELAY_ON);
        HAL_GPIO_WritePin(SOLENOID_VALVE_PORT, SOLENOID_VALVE_PIN, RELAY_ON);

        break;
    }
}
/**
 * @brief  检查是否吸到药品
 * @note   根据压力值判断是否吸到药品，压力值在10-70kPa范围内表示已吸到
 * @param  无
 * @retval LIQUID_State 吸液状态
 */
LIQUID_State Check_Liquid_Sucked(void)
{
    // 检查压力值是否在有效范围内
    if (g_pressure_value > PRESSURE_THRESHOLD_LOW && g_pressure_value < PRESSURE_THRESHOLD_HIGH)
    {
        liquid_state = LIQUID_SUCKED; // 已吸到药品
    }
    else
    {
        liquid_state = LIQUID_NOT_SUCKED; // 未吸到药品
    }
    
    return liquid_state;
}