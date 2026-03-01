#ifndef __PUMP_CONTROL_H
#define __PUMP_CONTROL_H

// 气泵继电器引脚定义（请根据实际硬件修改）
#define PUMP_RELAY_PIN      GPIO_PIN_13   // 示例：PB3
#define PUMP_RELAY_PORT     GPIOE

#define SOLENOID_VALVE_PIN  GPIO_PIN_9   // 电磁阀引脚
#define SOLENOID_VALVE_PORT GPIOE

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
void Pump_Update(void);              // 根据状态更新继电器（需周期性调用）

#endif /* __PUMP_CONTROL_H */