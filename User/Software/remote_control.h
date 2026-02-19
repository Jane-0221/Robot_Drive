#ifndef __REMOTE_CONTROL__
#define __REMOTE_CONTROL__

#include "stdint.h"
#include "main.h"

#define LOW_VALUE    353
#define MID_VALUE   1024
#define HIGH_VALUE  1694
#define RANGE        50


 typedef enum ARM_CONNECT_STATUS
 {
    ARM_CONNECT_STATUS_DISCONNECTED =0,
    ARM_CONNECT_STATUS_CONNECTED,
   
   
 } ARM_CONNECT_STATUS;
 extern ARM_CONNECT_STATUS arm_connect_status;
typedef enum BOOM_ARM_Stats{//机械臂气泵 
     ARM_BOOM_ON = 0,//机械臂气泵开 
     ARM_BOOM_OFF,//机械臂气泵关 

 }BOOM_ARM_Stats;
extern BOOM_ARM_Stats boom_arm_status; 

typedef enum BOOM_STORAGE_Stats{ //储矿气泵 
 STORAGE_OFF=0,//储矿气泵全关 
 STORAGE_ON,//储矿气泵全开 
 STORAGE1_ON,//储矿1气泵开，2关 
 STORAGE2_ON,//储矿2气泵开，1关 
}BOOM_STORAGE_Stats;
extern BOOM_STORAGE_Stats boom_storage_status;
extern float total_angle;
extern int sum_arm;
/*外部函数调用*/
void Motor_Forward(void);
void Motor_Reverse(void);
void Motor_Stop(void);





void remote_control_init(void);
void key_mouse_control(void);
void RC_Control(void);
void ARM_CONNECT_STATUS_UPDATA(void);
void start(void);
void disable_all(void);
void YAWFLOW_POS(void);
void enable_all(void);
void GPIO_init(void);
void PWM_control_init(void);
void switch_servos(void);
#endif // !__REMOTE_CONTROL__
