#include "arm_sv.h"
#include "bsp_pca9685.h"
#include "stdio.h"
// ��¼ÿ������ĵ�ǰռ�ձ�
static float current_duties[ARM_SV_COUNT];
ARM_SV_Duties_t duties_rx;

ARM_SV_Duties_t duties_tx;
// �ڲ�ͨ�����ú���
static void set_sv_duty(uint8_t id, float duty)
{
    if (id >= ARM_SV_COUNT)
        return;
    PCA9685_SetDuty(id, duty); // �ײ�ռ�ձ�����
    current_duties[id] = duty;
}

void ARM_SV_Init(float freq)
{
    duties_tx.duty0 = 0;
    duties_tx.duty1 = 0;
    duties_tx.duty2 = 0;
    duties_tx.duty3 = 0;
    duties_tx.duty4 = 0;
    duties_tx.duty5 = 0;
    // ��ʼ��PCA9685�ײ㣬����Ƶ�ʣ�����ͨ��ռ�ձ�Ϊ0
    PCA9685_Init(freq);

    // ��ʼ����¼����Ϊ0
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

    ARM_SV_SetDuty0(duties_tx.duty0); // ���0��
    ARM_SV_SetDuty1(duties_tx.duty1); // ���1��
    ARM_SV_SetDuty2(duties_tx.duty2); // ���2
    ARM_SV_SetDuty3(duties_tx.duty3); // ���3��
    ARM_SV_SetDuty4(duties_tx.duty4); // ���4��
    ARM_SV_SetDuty5(duties_tx.duty5); // ���5��
    
    // 获取所有舵机当前占空比
    duties_rx = ARM_SV_GetAllDuties();

    // 调试输出（已注释）
    // printf("===== Current PWM Duties =====\n");
    // printf("Servo 0: %.2f%%\n", duties_rx.duty0 * 100);
    // printf("Servo 1: %.2f%%\n", duties_rx.duty1 * 100);
    // printf("Servo 2: %.2f%%\n", duties_rx.duty2 * 100);
    // printf("Servo 3: %.2f%%\n", duties_rx.duty3 * 100);
    // printf("Servo 4: %.2f%%\n", duties_rx.duty4 * 100);
    // printf("Servo 5: %.2f%%\n", duties_rx.duty5 * 100);
}