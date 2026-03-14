#ifndef __REMOTE_CONTROL__
#define __REMOTE_CONTROL__

#include "stdint.h"
#include "main.h"
void remote_control_init(void);

extern void Pump_Control_Updata(void);
extern void Head_Motor_Control_Updata(void);
void Up_Down_Motor_Control_Updata(void);

// PC控制函数声明
extern void PC_Pump_Control_Updata(void);
extern void PC_Head_Motor_Control_Updata(void);
extern void PC_Up_Down_Motor_Control_Updata(void);

#endif // !__REMOTE_CONTROL__