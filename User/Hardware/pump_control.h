#ifndef __PUMP_CONTROL_H
#define __PUMP_CONTROL_H

// 气泵继电器引脚定义（请根据实际硬件修改）
#define PUMP_RELAY_PIN      GPIO_PIN_3   // 示例：PB3
#define PUMP_RELAY_PORT     GPIOB

// 继电器电平定义：低电平吸合，高电平断开（与之前定义一致）
#define RELAY_ON            GPIO_PIN_RESET
#define RELAY_OFF           GPIO_PIN_SET

// 气泵状态枚举
typedef enum {
    PUMP_OFF = 0,   // 气泵关闭
    PUMP_ON  = 1    // 气泵开启
} PUMP_State;

// 全局变量声明（若需与现有 boom_arm_status 关联，可注释掉下面一行并改用 extern）
extern PUMP_State pump_state;   // 当前气泵状态

// 函数声明
void Pump_Init(void);               // 初始化气泵
void Pump_On(void);                  // 开启气泵
void Pump_Off(void);                 // 关闭气泵
void Pump_SetState(PUMP_State state);// 设置指定状态
PUMP_State Pump_GetState(void);      // 获取当前状态
void Pump_Update(void);              // 根据状态更新继电器（需周期性调用）

#endif /* __PUMP_CONTROL_H */