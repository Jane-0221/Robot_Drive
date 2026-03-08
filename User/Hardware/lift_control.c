#include "lift_control.h" 
 #include "remote_control.h" 
 #include "usart.h" 
 #include "UART_data_txrx.h" 
 #include "stp23l.h" 
 int16_t aim_tx_height = 0; // 目标高度值，每帧更新 
 // 全局变量定义 
 LIFT_State lift_state = LIFT_STOP; // 初始状态为停止 
 uint16_t lift_current_height = 0;  // 初始高度为0

// 高度跟踪变量 
 int16_t lift_height_final = 0; // 每帧最终高度值，每帧更新 
 // 私有电机控制函数 
 static void Motor_Forward(void); 
 static void Motor_Reverse(void); 
 static void Motor_Stop(void);

/** 
  * @brief  初始化升降控制系统 
  * @note   确保升降机处于安全状态，初始化状态机并清除之前的状态 
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
  * @brief  设置升降机上升状态 
  * @param  无 
  * @retval 无 
  */
void Lift_Up(void)
{
    lift_state = LIFT_UP;
}

/** 
  * @brief  设置升降机下降状态 
  * @param  无 
  * @retval 无 
  */
void Lift_Down(void)
{
    lift_state = LIFT_DOWN;
}

/** 
  * @brief  设置升降机停止状态 
  * @param  无 
  * @retval 无 
  */
void Lift_Stop(void)
{
    lift_state = LIFT_STOP;
}

/** 
  * @brief  设置升降机指定状态 
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
  * @note   周期性调用（每10ms）来执行实际的电机控制 
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
  * @brief  从传感器刷新高度值 
  * @note   周期性调用以从传感器更新高度，使用STP23L传感器数据 
  * @param  无 
  * @retval 无 
  */
void Lift_RefreshHeight(void)
{

    if (stp23l_data.parse_ok == 1) 
     { 
         lift_height_final = STP23L_GetFinalDistPerFrame(); // Get height from sensor 
         STP23L_ClearOkFlag();                              // Clear flag to maintain consistency 
     } 
     // Example code for ADC height measurement (commented out) 
     // lift_current_height = HAL_ADC_GetValue(&hadc1); 
     // Currently using STP23L sensor for height measurement
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
    if (lift_height_final == 0||target_height == 0)
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