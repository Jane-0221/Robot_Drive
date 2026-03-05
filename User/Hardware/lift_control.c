#include "lift_control.h"
#include "remote_control.h"
#include "usart.h"
#include "UART_data_txrx.h"
#include "stp23l.h"
// 锟斤拷锟斤拷全锟街憋拷锟斤拷
LIFT_State lift_state = LIFT_STOP; // 锟斤拷始为停止
uint16_t lift_current_height = 0;  // 锟斤拷始锟竭讹拷0

// 锟斤拷锟斤拷植锟斤拷锟斤拷锟�
int16_t lift_height_final = 0; // 锟斤拷锟斤拷锟斤拷锟斤拷锟秸高讹拷值锟斤拷每帧锟斤拷锟斤拷一锟轿ｏ拷
// 锟阶诧拷锟斤拷锟斤拷锟狡猴拷锟斤拷锟斤拷锟节诧拷使锟矫ｏ拷
static void Motor_Forward(void);
static void Motor_Reverse(void);
static void Motor_Stop(void);

/**
 * @brief  锟斤拷始锟斤拷锟斤拷锟斤拷锟剿匡拷锟斤拷
 * @note   确锟斤拷锟斤拷锟酵Ｖ癸拷锟阶刺拷锟轿伙拷锟斤拷锟斤拷懦锟绞硷拷锟酵拷锟斤拷锟接诧拷锟斤拷锟绞硷拷锟斤拷锟斤拷锟斤拷
 * @param  锟斤拷
 * @retval 锟斤拷
 */
void Lift_Init(void)
{
    Motor_Stop();
    lift_state = LIFT_STOP;
    lift_current_height = 0;
}

/**
 * @brief  锟斤拷锟斤拷锟斤拷锟斤拷状态
 * @param  锟斤拷
 * @retval 锟斤拷
 */
void Lift_Up(void)
{
    lift_state = LIFT_UP;
}

/**
 * @brief  锟斤拷锟斤拷锟铰斤拷状态
 * @param  锟斤拷
 * @retval 锟斤拷
 */
void Lift_Down(void)
{
    lift_state = LIFT_DOWN;
}

/**
 * @brief  锟斤拷锟斤拷停止状态
 * @param  锟斤拷
 * @retval 锟斤拷
 */
void Lift_Stop(void)
{
    lift_state = LIFT_STOP;
}

/**
 * @brief  锟斤拷锟斤拷指锟斤拷状态
 * @param  state 要锟斤拷锟矫碉拷状态
 * @retval 锟斤拷
 */
void Lift_SetState(LIFT_State state)
{
    lift_state = state;
}

/**
 * @brief  锟斤拷取锟斤拷前状态
 * @retval 锟斤拷前状态
 */
LIFT_State Lift_GetState(void)
{
    return lift_state;
}

/**
 * @brief  锟斤拷锟捷碉拷前状态锟斤拷锟铰碉拷锟斤拷锟斤拷锟�
 * @note   锟剿猴拷锟斤拷锟斤拷锟斤拷锟斤拷锟皆碉拷锟矫ｏ拷锟斤拷锟斤拷每10ms锟斤拷锟斤拷锟斤拷执锟斤拷实锟绞匡拷锟斤拷
 * @param  锟斤拷
 * @retval 锟斤拷
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
 * @brief  刷锟铰高讹拷值锟斤拷锟接达拷锟斤拷锟斤拷锟斤拷取锟斤拷
 * @note   锟矫伙拷锟斤拷锟斤拷锟绞碉拷锟接诧拷锟斤拷诖锟斤拷锟斤拷佣锟饺★拷锟斤拷耄拷锟斤拷锟紸DC锟斤拷锟斤拷锟斤拷锟�
 * @param  锟斤拷
 * @retval 锟斤拷
 */
void Lift_RefreshHeight(void)
{

    if (stp23l_data.parse_ok == 1)
    {
        lift_height_final = STP23L_GetFinalDistPerFrame(); // 锟斤拷锟侥碉拷锟斤拷
        STP23L_ClearOkFlag();                              // 锟斤拷锟斤拷锟街撅拷锟阶硷拷锟斤拷锟揭恢★拷锟斤拷锟�
    }
    // 示锟斤拷锟斤拷锟斤拷锟斤拷叨锟斤拷锟紸DC锟缴硷拷锟斤拷通锟斤拷1
    // lift_current_height = HAL_ADC_GetValue(&hadc1);
    // 锟剿达拷锟斤拷锟秸ｏ拷锟斤拷锟斤拷锟绞碉拷锟接诧拷锟绞碉拷锟�
}

/**
 * @brief  锟斤拷取锟斤拷前锟竭讹拷
 * @retval 锟斤拷前锟竭讹拷值
 */
uint16_t Lift_GetHeight(void)
{
    return lift_current_height;
}

/* **************** 锟阶诧拷锟斤拷锟斤拷锟狡ｏ拷锟斤拷锟斤拷锟结供锟侥达拷锟斤拷一锟铰ｏ拷 **************** */

/**
 * @brief  锟斤拷锟斤拷锟阶拷锟斤拷锟斤拷锟斤拷锟�
 * @note   IN1(PA0)=锟酵碉拷平锟斤拷锟较ｏ拷IN2(PA2)=锟竭碉拷平锟较匡拷 锟斤拷 锟斤拷锟斤拷锟阶�
 * @param  锟斤拷
 * @retval 锟斤拷
 */
static void Motor_Forward(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_ON);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_OFF);
}

/**
 * @brief  锟斤拷锟斤拷锟阶拷锟斤拷陆锟斤拷锟�
 * @note   IN1(PA0)=锟竭碉拷平锟较匡拷锟斤拷IN2(PA2)=锟酵碉拷平锟斤拷锟斤拷 锟斤拷 锟斤拷锟斤拷锟阶�
 * @param  锟斤拷
 * @retval 锟斤拷
 */
static void Motor_Reverse(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_OFF);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_ON);
}

/**
 * @brief  锟斤拷锟酵Ｖ�
 * @note   锟斤拷路锟教碉拷锟斤拷锟斤拷锟较匡拷锟斤拷锟竭碉拷平锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟斤拷锟�
 * @param  锟斤拷
 * @retval 锟斤拷
 */
static void Motor_Stop(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_ON);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_ON);
}

/**
 * @brief  控制到目标高度
 * @note   通过遥控器设置lift_height_final达到目标位置，误差5以内停止
 * @param  target_height 目标高度值
 * @retval 无
 */
void Lift_GoToTarget(int16_t target_height)
{
    // 刷新高度值
    Lift_RefreshHeight();
    if (lift_height_final == 0)
    {
        return;
    }
    // 计算目标高度和当前高度的差值
    int16_t height_diff = target_height - lift_height_final;

    // 根据差值设置升降机运动状态
    if (height_diff > 5)
    {
        // 目标高度高于当前高度，向上升
        lift_state = LIFT_UP;
        return;
    }
    else if (height_diff < -5)
    {
        // 目标高度低于当前高度，向下降
        lift_state = LIFT_DOWN;
        return;
    }
    else
    {
        // 已经达到目标高度，停止
        lift_state = LIFT_STOP;
        return;
    }
}