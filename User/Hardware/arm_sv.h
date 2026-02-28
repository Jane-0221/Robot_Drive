#ifndef __ARM_SV_H
#define __ARM_SV_H

#include <stdint.h>

// 机械臂使用的舵机数量
#define ARM_SV_COUNT        6

// 用于保存6个舵机占空比的结构体
typedef struct {
    float duty0;   // 舵机0占空比
    float duty1;   // 舵机1占空比
    float duty2;   // 舵机2占空比
    float duty3;   // 舵机3占空比
    float duty4;   // 舵机4占空比
    float duty5;   // 舵机5占空比
} ARM_SV_Duties_t;

extern ARM_SV_Duties_t duties_tx;
// 初始化机械臂舵机控制（设置PWM频率）
// freq : PWM频率（Hz），通常为50
void ARM_SV_Init(float freq);

// 设置指定舵机的PWM占空比（通用函数）
// sv_id : 舵机ID (0 ~ ARM_SV_COUNT-1)
// duty  : 占空比 (0.0 ~ 1.0)
void ARM_SV_SetDuty(uint8_t sv_id, float duty);

// 六个独立舵机占空比设置函数
void ARM_SV_SetDuty0(float duty);
void ARM_SV_SetDuty1(float duty);
void ARM_SV_SetDuty2(float duty);
void ARM_SV_SetDuty3(float duty);
void ARM_SV_SetDuty4(float duty);
void ARM_SV_SetDuty5(float duty);

// 获取指定舵机当前占空比（软件记录值）
float ARM_SV_GetDuty(uint8_t sv_id);

// 获取六个独立舵机当前占空比
float ARM_SV_GetDuty0(void);
float ARM_SV_GetDuty1(void);
float ARM_SV_GetDuty2(void);
float ARM_SV_GetDuty3(void);
float ARM_SV_GetDuty4(void);
float ARM_SV_GetDuty5(void);

// 批量设置所有舵机占空比
// duties : 长度为ARM_SV_COUNT的数组，包含每个舵机的占空比
void ARM_SV_SetAllDuties(const float *duties);
// 获取所有舵机占空比，返回结构体
ARM_SV_Duties_t ARM_SV_GetAllDuties(void);
extern void ARM_SV_Tx_Rx(void);
#endif