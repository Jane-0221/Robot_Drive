#include "head.h"
#include "ktech_motor.h"
KTech_Motor_t motor_linkong[2];// 电机反馈帧结构体定义
Head_MotorData_t head_motor_data[2];// 头电机数据结构体定义



void Head_Init()
{
    ktech_motor_init(MOTOR_LINKONG_1_ID);
    // 2. 开启电机（从关闭状态切换到开启状态）
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_1_ID);

    //电机发送数据初始化
    head_motor_data[0].direction = DIR_CW;   //0:顺时针, 1:逆时针
    head_motor_data[0].target_angle = 18000;
    head_motor_data[0].max_speed = 360;

    ktech_motor_init(MOTOR_LINKONG_2_ID);
    // 2. 开启电机（从关闭状态切换到开启状态）
    ktech_motor_on(CAN_HANDLE_1, MOTOR_LINKONG_2_ID);
     //电机发送数据初始化
    head_motor_data[1].direction = DIR_CW;   //0:顺时针, 1:逆时针
    head_motor_data[1].target_angle = 9000;
    head_motor_data[1].max_speed = 360;


};
// 循环执行的电机控制函数
void Head_Lk_motor1(void)
{

    // 静态变量：仅第一次执行时初始化，后续永久保留上一次的目标角度值，用于新老对比
    // 注意：如果您的target_angle是int32_t类型，此处同步改为int32_t
    static uint32_t last_target_angle = 0;
    // 提取本次新的目标角度，简化代码可读性
    uint32_t current_target = head_motor_data[0].target_angle;

    /************************* 核心方向判断逻辑 *************************/
    // 新目标值 > 上一次的老值 → 顺时针，方向设为0
    if (current_target > last_target_angle)
    {
        head_motor_data[0].direction = DIR_CW;
    }
    // 新目标值 < 上一次的老值 → 逆时针，方向设为1
    else if (current_target < last_target_angle)
    {
        head_motor_data[0].direction = DIR_CCW;
    }
    // 新老值相等时：方向保持不变，不做修改，避免无意义的参数刷新和电机抖动

    /************************* 执行电机控制 *************************/
    ktech_pos_single2(CAN_HANDLE_1, 
                      MOTOR_LINKONG_1_ID, 
                      head_motor_data[0].direction, 
                      current_target, 
                      head_motor_data[0].max_speed);

    /************************* 更新老值，供下一次循环对比 *************************/
    last_target_angle = current_target;
}
void Head_Lk_motor2()
{
    
    // 静态变量：仅第一次执行时初始化，后续永久保留上一次的目标角度值，用于新老对比
    // 注意：如果您的target_angle是int32_t类型，此处同步改为int32_t
    static uint32_t last_target_angle = 0;
    // 提取本次新的目标角度，简化代码可读性
    uint32_t current_target = head_motor_data[1].target_angle;

    /************************* 核心方向判断逻辑 *************************/
    // 新目标值 > 上一次的老值 → 顺时针，方向设为0
    if (current_target > last_target_angle)
    {
        head_motor_data[1].direction = DIR_CW;
    }
    // 新目标值 < 上一次的老值 → 逆时针，方向设为1
    else if (current_target < last_target_angle)
    {
        head_motor_data[1].direction = DIR_CCW;
    }
    // 新老值相等时：方向保持不变，不做修改，避免无意义的参数刷新和电机抖动

    /************************* 执行电机控制 *************************/
    ktech_pos_single2(CAN_HANDLE_1, 
                      MOTOR_LINKONG_2_ID, 
                      head_motor_data[1].direction, 
                      current_target, 
                      head_motor_data[1].max_speed);

    /************************* 更新老值，供下一次循环对比 *************************/
    last_target_angle = current_target;
}
void Head_all_tx()
{
Head_Lk_motor1();
HAL_Delay(1);
Head_Lk_motor2();

}
void Arm_Lk_Data_update()
{

head_motor_data[0].current_angle=motor_linkong[0].fb.encoder/65536.0f * 360.0f;     // 反馈编码器值转换为角度值
head_motor_data[0].current_velocity=motor_linkong[0].fb.speed;                 // 反馈速度值转换为角度值

head_motor_data[1].current_angle=motor_linkong[1].fb.encoder/65536.0f * 360.0f;     // 反馈编码器值转换为角度值
head_motor_data[1].current_velocity=motor_linkong[1].fb.speed;                 // 反馈速度值转换为角度值


}