#ifndef __REMOTE_CONTROL__
#define __REMOTE_CONTROL__

#include "stdint.h"
#include "main.h"

// 定义气泵状态
typedef enum BOOM_ARM_Stats {   // 机械臂气泵
    ARM_BOOM_ON = 0,             // 机械臂气泵开
    ARM_BOOM_OFF,                // 机械臂气泵关
} BOOM_ARM_Stats;
extern BOOM_ARM_Stats boom_arm_status;
//定义升降杆状态
typedef enum LIFT_State {
    LIFT_UP = 0,    // 升降上升
    LIFT_DOWN,      // 升降下降
    LIFT_STOP       // 升降停止
} LIFT_State;
extern LIFT_State lift_state;
/* 外部函数调用 */
void Motor_Forward(void);
void Motor_Reverse(void);
void Motor_Stop(void);
void Up_Down_Motor_Control_Updata(void);
void Up_Down_Motor_Control(void);
void remote_control_init(void);
void GPIO_init(void);

#endif // !__REMOTE_CONTROL__