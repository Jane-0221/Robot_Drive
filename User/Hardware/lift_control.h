#ifndef __LIFT_CONTROL_H
#define __LIFT_CONTROL_H
#include "stdint.h"
#include "main.h"

// 继电器引脚定义（与您提供的代码一致）
#define RELAY1_PIN          GPIO_PIN_0
#define RELAY1_PORT         GPIOA
#define RELAY2_PIN          GPIO_PIN_2
#define RELAY2_PORT         GPIOA

// 继电器电平定义：低电平吸合，高电平断开
#define RELAY_ON            GPIO_PIN_RESET
#define RELAY_OFF           GPIO_PIN_SET

//定义升降杆状态
typedef enum LIFT_State {
    LIFT_UP = 0,    // 升降上升
    LIFT_DOWN,      // 升降下降
    LIFT_STOP       // 升降停止
} LIFT_State;
extern LIFT_State lift_state;


extern uint16_t lift_current_height;   // 当前高度（单位可自定义）

// 函数声明
void Lift_Init(void);                          // 初始化升降杆模块
void Lift_Up(void);                             // 设置上升状态
void Lift_Down(void);                           // 设置下降状态
void Lift_Stop(void);                           // 设置停止状态
void Lift_SetState(LIFT_State state);           // 设置指定状态
LIFT_State Lift_GetState(void);                  // 获取当前状态
void Lift_UpdateMotor(void);                     // 根据状态更新电机（需周期性调用）
void Lift_RefreshHeight(void);                   // 刷新高度（从传感器读取）
uint16_t Lift_GetHeight(void);                    // 获取当前高度


#endif /* __LIFT_CONTROL_H */