#include "arm_sv.h"
#include "bsp_pca9685.h"
#include "stdio.h"
// 记录每个舵机的当前占空比
static float current_duties[ARM_SV_COUNT];
ARM_SV_Duties_t duties_rx;
// 内部通用设置函数
static void set_sv_duty(uint8_t id, float duty)
{
    if (id >= ARM_SV_COUNT)
        return;
    PCA9685_SetDuty(id, duty); // 底层占空比设置
    current_duties[id] = duty;
}

void ARM_SV_Init(float freq)
{
    // 初始化PCA9685底层，设置频率，所有通道占空比为0
    PCA9685_Init(freq);

    // 初始化记录数组为0
    for (uint8_t i = 0; i < ARM_SV_COUNT; i++)
    {
        current_duties[i] = 0.0f;
    }
}

void ARM_SV_SetDuty(uint8_t sv_id, float duty)
{
    set_sv_duty(sv_id, duty);
}

void ARM_SV_SetDuty0(float duty) { set_sv_duty(0, duty); }
void ARM_SV_SetDuty1(float duty) { set_sv_duty(1, duty); }
void ARM_SV_SetDuty2(float duty) { set_sv_duty(2, duty); }
void ARM_SV_SetDuty3(float duty) { set_sv_duty(3, duty); }
void ARM_SV_SetDuty4(float duty) { set_sv_duty(4, duty); }
void ARM_SV_SetDuty5(float duty) { set_sv_duty(5, duty); }

float ARM_SV_GetDuty(uint8_t sv_id)
{
    if (sv_id >= ARM_SV_COUNT)
        return 0.0f;
    return current_duties[sv_id];
}

float ARM_SV_GetDuty0(void) { return current_duties[0]; }
float ARM_SV_GetDuty1(void) { return current_duties[1]; }
float ARM_SV_GetDuty2(void) { return current_duties[2]; }
float ARM_SV_GetDuty3(void) { return current_duties[3]; }
float ARM_SV_GetDuty4(void) { return current_duties[4]; }
float ARM_SV_GetDuty5(void) { return current_duties[5]; }

void ARM_SV_SetAllDuties(const float *duties)
{
    for (uint8_t i = 0; i < ARM_SV_COUNT; i++)
    {
        ARM_SV_SetDuty(i, duties[i]);
    }
}

ARM_SV_Duties_t ARM_SV_GetAllDuties(void)
{
    ARM_SV_Duties_t duties;

    duties.duty0 = current_duties[0];
    duties.duty1 = current_duties[1];
    duties.duty2 = current_duties[2];
    duties.duty3 = current_duties[3];
    duties.duty4 = current_duties[4];
    duties.duty5 = current_duties[5];

    return duties;
}
void ARM_SV_Tx_Rx()
{
    float duties[6] = {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f};
    ARM_SV_SetAllDuties(duties);
    duties_rx = ARM_SV_GetAllDuties();
        printf("===== Current PWM Duties =====\n");
    printf("Servo 0: %.2f%%\n", duties_rx.duty0 * 100);
    printf("Servo 1: %.2f%%\n", duties_rx.duty1 * 100);
    printf("Servo 2: %.2f%%\n", duties_rx.duty2 * 100);
    printf("Servo 3: %.2f%%\n", duties_rx.duty3 * 100);
    printf("Servo 4: %.2f%%\n", duties_rx.duty4 * 100);
    printf("Servo 5: %.2f%%\n", duties_rx.duty5 * 100);
}