#include "remote_control.h"
#include "ramp_generator.h"
#include "IMU_updata.h"
#include "Stm32_time.h"
#include <tim.h>
#include "pid.h"
#include "UART_data_txrx.h"
#include "Sbus.h"
#include "head.h"
#include "arm.h"
#include "lift_control.h"
#include "pump_control.h"
// 遥控器值
#define LOW_VALUE 353
#define MID_VALUE 1024
#define HIGH_VALUE 1694
#define RANGE 50



void remote_control_init()
{

}


void Head_Motor_Control_Updata(void)
{
if(SBUS_CH.CH6==HIGH_VALUE)
{
    Daran_motor_data[0].target_angle=0;
}
else if(SBUS_CH.CH6==MID_VALUE)
{
    Daran_motor_data[0].target_angle=90;
}
else if(SBUS_CH.CH6==LOW_VALUE)
{
    Daran_motor_data[0].target_angle=180;
}

}

void Up_Down_Motor_Control(void)
{

    switch (SBUS_CH.CH7)
    {
    case HIGH_VALUE:
        lift_state = LIFT_UP;
        break;
    case LOW_VALUE:
        lift_state = LIFT_DOWN;
         break;
    case MID_VALUE:
    default: // 兜底，确保任何情况都有处理
        lift_state = LIFT_STOP;
        break;
    }
}



