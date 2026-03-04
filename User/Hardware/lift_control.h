#ifndef __LIFT_CONTROL_H
#define __LIFT_CONTROL_H
#include "stdint.h"
#include "main.h"

// �̵������Ŷ��壨�����ṩ�Ĵ���һ�£�
#define RELAY1_PIN          GPIO_PIN_0
#define RELAY1_PORT         GPIOA
#define RELAY2_PIN          GPIO_PIN_2
#define RELAY2_PORT         GPIOA

// �̵�����ƽ���壺�͵�ƽ���ϣ��ߵ�ƽ�Ͽ�
#define RELAY_ON            GPIO_PIN_SET
#define RELAY_OFF           GPIO_PIN_RESET

//����������״̬
typedef enum LIFT_State {
    LIFT_UP = 0,    // ��������
    LIFT_DOWN,      // �����½�
    LIFT_STOP       // ����ֹͣ
} LIFT_State;
extern LIFT_State lift_state;


extern uint16_t lift_current_height;   // ��ǰ�߶ȣ���λ���Զ��壩

// ��������
void Lift_Init(void);                          // ��ʼ��������ģ��
void Lift_Up(void);                             // ��������״̬
void Lift_Down(void);                           // �����½�״̬
void Lift_Stop(void);                           // ����ֹͣ״̬
void Lift_SetState(LIFT_State state);           // ����ָ��״̬
LIFT_State Lift_GetState(void);                  // ��ȡ��ǰ״̬
void Lift_UpdateMotor(void);                     // ����״̬���µ�����������Ե��ã�?
void Lift_RefreshHeight(void);                   // ˢ�¸߶ȣ��Ӵ�������ȡ��
uint16_t Lift_GetHeight(void);                    // ��ȡ��ǰ�߶�
void Lift_GoToTarget(int16_t target_height);    // ���Ƶ�Ŀ��߶�


#endif /* __LIFT_CONTROL_H */