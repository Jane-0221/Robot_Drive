#include "lift_control.h"
#include "remote_control.h"
#include "usart.h"
#include "UART_data_txrx.h"
#include "stp23l.h"
// ����ȫ�ֱ���
LIFT_State lift_state = LIFT_STOP;      // ��ʼΪֹͣ
uint16_t lift_current_height = 0;       // ��ʼ�߶�0

//����ֲ�����
int16_t lift_height_final = 0; // ���������ո߶�ֵ��ÿ֡����һ�Σ�
// �ײ������ƺ������ڲ�ʹ�ã�
static void Motor_Forward(void);
static void Motor_Reverse(void);
static void Motor_Stop(void);

/**
 * @brief  ��ʼ�������˿���
 * @note   ȷ�����ֹͣ��״̬��λ�����ų�ʼ��ͨ����Ӳ����ʼ�������
 * @param  ��
 * @retval ��
 */
void Lift_Init(void)
{
    Motor_Stop();
    lift_state = LIFT_STOP;
    lift_current_height = 0;
}

/**
 * @brief  ��������״̬
 * @param  ��
 * @retval ��
 */
void Lift_Up(void)
{
    lift_state = LIFT_UP;
}

/**
 * @brief  �����½�״̬
 * @param  ��
 * @retval ��
 */
void Lift_Down(void)
{
    lift_state = LIFT_DOWN;
}

/**
 * @brief  ����ֹͣ״̬
 * @param  ��
 * @retval ��
 */
void Lift_Stop(void)
{
    lift_state = LIFT_STOP;
}

/**
 * @brief  ����ָ��״̬
 * @param  state Ҫ���õ�״̬
 * @retval ��
 */
void Lift_SetState(LIFT_State state)
{
    lift_state = state;
}

/**
 * @brief  ��ȡ��ǰ״̬
 * @retval ��ǰ״̬
 */
LIFT_State Lift_GetState(void)
{
    return lift_state;
}

/**
 * @brief  ���ݵ�ǰ״̬���µ������
 * @note   �˺����������Ե��ã�����ÿ10ms������ִ��ʵ�ʿ���
 * @param  ��
 * @retval ��
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
 * @brief  ˢ�¸߶�ֵ���Ӵ�������ȡ��
 * @note   �û������ʵ��Ӳ���ڴ����Ӷ�ȡ���룬����ADC�������
 * @param  ��
 * @retval ��
 */
void Lift_RefreshHeight(void)
{

            if(stp23l_data.parse_ok == 1)
        {
            lift_height_final = STP23L_GetFinalDistPerFrame(); // ���ĵ���
            STP23L_ClearOkFlag(); // �����־��׼����һ֡����
        }
    // ʾ��������߶���ADC�ɼ���ͨ��1
    // lift_current_height = HAL_ADC_GetValue(&hadc1);
    // �˴����գ������ʵ��Ӳ��ʵ��
}

/**
 * @brief  ��ȡ��ǰ�߶�
 * @retval ��ǰ�߶�ֵ
 */
uint16_t Lift_GetHeight(void)
{
    return lift_current_height;
}

/* **************** �ײ������ƣ������ṩ�Ĵ���һ�£� **************** */

/**
 * @brief  �����ת��������
 * @note   IN1(PA0)=�͵�ƽ���ϣ�IN2(PA2)=�ߵ�ƽ�Ͽ� �� �����ת
 * @param  ��
 * @retval ��
 */
static void Motor_Forward(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_ON);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_OFF);
}

/**
 * @brief  �����ת���½���
 * @note   IN1(PA0)=�ߵ�ƽ�Ͽ���IN2(PA2)=�͵�ƽ���� �� �����ת
 * @param  ��
 * @retval ��
 */
static void Motor_Reverse(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_OFF);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_ON);
}

/**
 * @brief  ���ֹͣ
 * @note   ��·�̵������Ͽ����ߵ�ƽ���������������
 * @param  ��
 * @retval ��
 */
static void Motor_Stop(void)
{
    HAL_GPIO_WritePin(RELAY1_PORT, RELAY1_PIN, RELAY_ON);
    HAL_GPIO_WritePin(RELAY2_PORT, RELAY2_PIN, RELAY_ON);
}