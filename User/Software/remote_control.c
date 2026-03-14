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
#include "arm_sv.h"
#include "uart_protocol.h" // 添加PC通信协议头文件
// 遥控器值
#define LOW_VALUE 353
#define MID_VALUE 1024
#define HIGH_VALUE 1694
#define RANGE 50
#define PI 3.14159265358979323846f
#define THREE_PI_OVER_FOUR (PI * 3.0f / 4.0f)
float motor_radians[6] = {0.0f};
void remote_control_init()
{
}
/**
 * @brief 270°模式下弧度转占空比函数
 * @param radian 输入弧度值（范围：-3π/4 ~ +3π/4，即-135°~+135°）
 * @return 对应的占空比值（范围：0.025 ~ 0.125）
 * @note 占空比0.075对应弧度0，线性映射关系
 */
float radian_to_duty_270(float radian)
{
    // 角度范围：-135° ~ +135°（-3π/4 ~ +3π/4 弧度）
    // 占空比范围：0.025 ~ 0.125
    // 中点：弧度0对应占空比0.075

    // 限制弧度范围
    if (radian < -THREE_PI_OVER_FOUR)
    {
        radian = -THREE_PI_OVER_FOUR;
    }
    else if (radian > THREE_PI_OVER_FOUR)
    {
        radian = THREE_PI_OVER_FOUR;
    }

    // 线性映射公式：duty = 0.075 + (radian / (3π/4)) * 0.05
    float duty = 0.075f + (radian / THREE_PI_OVER_FOUR) * 0.05f;

    return duty;
}
/**
 * @brief 设置6个电机的弧度值
 * @param radians 包含6个弧度值的数组
 * @note 每个弧度值范围：-3π/4 ~ +3π/4
 */
void set_motor_radians_270(float radians[6])
{
    for (int i = 0; i < 6; i++)
    {
        // 将弧度转换为占空比并设置
        switch (i)
        {
        case 0:
            duties_tx.duty0 = radian_to_duty_270(radians[i] / 2);
            break;
        case 1:
            duties_tx.duty1 = radian_to_duty_270(radians[i] / 2);
            break;
        case 2:
            duties_tx.duty2 = radian_to_duty_270(radians[i]);
            break;
        case 3:
            duties_tx.duty3 = radian_to_duty_270(radians[i]);
            break;
        case 4:
            duties_tx.duty4 = radian_to_duty_270(radians[i]);
            break;
        case 5:
            duties_tx.duty5 = radian_to_duty_270(radians[i]);
            break;
        }
    }
}
void Pump_Control_Updata(void)
{
    if (SBUS_CH.CH8 == HIGH_VALUE)
    {
        pump_state = PUMP_ON;
    }
    else if (SBUS_CH.CH8 == LOW_VALUE)
    {
        pump_state = PUMP_OFF;
    }
}

void Head_Motor_Control_Updata(void)
{
    // 替换原有if-else if结构为switch语句
    if(SBUS_CH.CH8==HIGH_VALUE)
    {
    switch (SBUS_CH.CH6)
    {
    case HIGH_VALUE:
        Daran_motor_data[0].target_angle = 0;
        Daran_motor_data[1].target_angle = 0;

        // duties_tx.duty0 = 0.12;
        //  duties_tx.duty1 = 0.12;
        //  duties_tx.duty2 = 0.12;
        //  duties_tx.duty3 = 0.12;
        //  duties_tx.duty4 = 0.12;
        //  duties_tx.duty5 = 0.12;
        // motor_radians[0] = PI / 4;
        // motor_radians[1] = PI / 3;
        // motor_radians[2] = PI / 2;
        // motor_radians[3] = PI / 2;
        // motor_radians[4] = PI / 2;
        // motor_radians[5] = PI / 2;
        if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
        {
            motor_radians[0] += 0.003f;
        }
        else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
        {
            motor_radians[0] -= 0.003f;
        }

        if (SBUS_CH.CH2 > (MID_VALUE + RANGE))
        {
            motor_radians[1] += 0.003f;
        }
        else if (SBUS_CH.CH2 < (MID_VALUE - RANGE))
        {
            motor_radians[1] -= 0.003f;
        }

        set_motor_radians_270(motor_radians);
        break;

    case MID_VALUE:
        Daran_motor_data[0].target_angle = 90;
        Daran_motor_data[1].target_angle = 90;

        // duties_tx.duty0 = 0.075;
        // duties_tx.duty1 = 0.075;
        // duties_tx.duty2 = 0.075;
        // duties_tx.duty3 = 0.075;
        // duties_tx.duty4 = 0.075;
        // duties_tx.duty5 = 0.075;
        // motor_radians[0] = 0.0f;
        // motor_radians[1] = 0.0f;
        // motor_radians[2] = 0.0f;
        // motor_radians[3] = 0.0f;
        // motor_radians[4] = 0.0f;
        // motor_radians[5] = 0.0f;
        if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
        {
            motor_radians[2] += 0.003f;
        }
        else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
        {
            motor_radians[2] -= 0.003f;
        }

        if (SBUS_CH.CH2 > (MID_VALUE + RANGE))
        {
            motor_radians[3] += 0.003f;
        }
        else if (SBUS_CH.CH2 < (MID_VALUE - RANGE))
        {
            motor_radians[3] -= 0.003f;
        }
        set_motor_radians_270(motor_radians);
        break;

    case LOW_VALUE:
        Daran_motor_data[0].target_angle = 180;
        Daran_motor_data[1].target_angle = 180;

        // duties_tx.duty0 = 0.03;
        // duties_tx.duty1 = 0.03;
        // duties_tx.duty2 = 0.03;
        // duties_tx.duty3 = 0.03;
        // duties_tx.duty4 = 0.03;
        // duties_tx.duty5 = 0.03;
        // motor_radians[0] = -PI / 4;
        // motor_radians[1] = -PI / 3;
        // motor_radians[2] = -PI / 2;
        // motor_radians[3] = -PI / 2;
        // motor_radians[4] = -PI / 4;
        // motor_radians[5] = -PI / 4;
        if (SBUS_CH.CH1 > (MID_VALUE + RANGE))
        {
            motor_radians[4] += 0.003f;
        }
        else if (SBUS_CH.CH1 < (MID_VALUE - RANGE))
        {
            motor_radians[4] -= 0.003f;
        }

        if (SBUS_CH.CH2 > (MID_VALUE + RANGE))
        {
            motor_radians[5] += 0.003f;
        }
        else if (SBUS_CH.CH2 < (MID_VALUE - RANGE))
        {
            motor_radians[5] -= 0.003f;
        }
        set_motor_radians_270(motor_radians);
        break;

    default:
        // 可选：处理SBUS_CH.CH6不是上述三个值的情况
        // 如果不需要特殊处理，这里可以留空
        break;
    }
}
else if(SBUS_CH.CH8==LOW_VALUE)
{
    switch (SBUS_CH.CH6)
    {
case HIGH_VALUE:  // 挥手到左边（第三张图的弧度值）
    motor_radians[0] = -0.101999983;
    motor_radians[1] = -0.29700008;
    motor_radians[2] = 1.80600858;
    motor_radians[3] = 2.40001273;
    motor_radians[4] = 0.0119999889;
    motor_radians[5] = -0.0150000118;
    break;

case MID_VALUE:   // 举手（中间姿态，第二张图的弧度值）
    motor_radians[0] = -0.101999983;
    motor_radians[1] = -0.29700008;
    motor_radians[2] = 1.79100847;
    motor_radians[3] = 1.88400912;
    motor_radians[4] = 0.0119999889;
    motor_radians[5] = -0.0150000118;
    break;

case LOW_VALUE:   // 挥手到右边（第一张图的弧度值）
    motor_radians[0] = -0.101999983;
    motor_radians[1] = -0.29700008;
    motor_radians[2] = 1.88700914;
    motor_radians[3] = 1.30800509;
    motor_radians[4] = 0.0119999889;
    motor_radians[5] = -0.0150000118;
    break;



    // 默认情况：保持当前关节角度（避免无操作时数组值异常）
    default:
        // 可选：如果需要默认姿态，可赋值为举手姿态
        // motor_radians[0] = -0.0450000018f;
        // ... 其他关节赋值
        break;

    }

    set_motor_radians_270(motor_radians);
}

}

void Up_Down_Motor_Control_Updata(void)
{
    switch (SBUS_CH.CH7)
    {
    case HIGH_VALUE:
        aim_tx_height = 100;
        break;
    case LOW_VALUE:
        aim_tx_height = 700;
        break;
    case MID_VALUE:
        aim_tx_height = 400;
        break;
    default:
        break;
    }

}

// PC控制函数实现

/**
 * @brief PC控制气泵更新函数
 * @note 使用PC传入的气泵状态信息进行控制
 */
void PC_Pump_Control_Updata(void)
{
    // 使用PC传入的气泵状态信息
    if (pc_dn_data.pc_pump_state == 1)
    {
        pump_state = PUMP_ON;
    }
    else if (pc_dn_data.pc_pump_state == 0)
    {
        pump_state = PUMP_OFF;
    }
}

/**
 * @brief PC控制头部电机更新函数
 * @note 使用PC传入的头部电机角度信息进行控制
 */
void PC_Head_Motor_Control_Updata(void)
{
    // 使用PC传入的头部电机角度信息（索引0和1对应头部电机）
    // pc_target_angles[0] 和 pc_target_angles[1] 是0.1度的单位，需要转换为度
    Daran_motor_data[0].target_angle = (float)pc_dn_data.pc_target_angles[0] / 10.0f;
    Daran_motor_data[1].target_angle = (float)pc_dn_data.pc_target_angles[1] / 10.0f;

    // 根据角度设置占空比（示例逻辑，可根据实际需求调整）
    float duty_base = 0.025f + (Daran_motor_data[0].target_angle / 180.0f) * 0.1f;

    duties_tx.duty0 = duty_base;
    duties_tx.duty1 = duty_base;
    duties_tx.duty2 = duty_base;
    duties_tx.duty3 = duty_base;
    duties_tx.duty4 = duty_base;
    duties_tx.duty5 = duty_base;
}

/**
 * @brief PC控制升降电机更新函数
 * @note 使用PC传入的升降目标高度信息进行控制
 */
void PC_Up_Down_Motor_Control_Updata(void)
{
    // 使用PC传入的升降目标高度信息（0.1mm单位）
    aim_tx_height = pc_dn_data.pc_target_lift_height;
}