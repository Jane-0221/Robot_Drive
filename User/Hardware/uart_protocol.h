#ifndef __UART_PROTOCOL_H
#define __UART_PROTOCOL_H

#include "stm32h7xx_hal.h"

// ===================== 串口协议帧格式相关宏定义 =====================
#define FRAME_HEADER1 0xAA       // 帧头第1字节（固定标识，用于帧同步）
#define FRAME_HEADER2 0x55       // 帧头第2字节（固定标识，用于帧同步）
#define FRAME_TAIL1   0xEE       // 帧尾第1字节（固定标识，用于帧结束）
#define FRAME_TAIL2   0xFF       // 帧尾第2字节（固定标识，用于帧结束）
#define UP_FRAME_TYPE 0x01       // 上行帧类型标识（STM32→PC的数据帧）
#define DN_FRAME_TYPE 0x02       // 下行帧类型标识（PC→STM32的数据帧）

// ===================== 串口协议数据段长度宏定义 =====================
#define UP_DATA_LEN   28         // 上行帧数据段固定长度（单位：字节）
#define DN_DATA_LEN   51         // 下行帧数据段固定长度（单位：字节）

// ===================== 上行数据结构体（STM32→PC） =====================
/**
 * @brief  上行数据结构体：STM32发送给PC的设备状态数据
 * @note   所有角度/高度单位均为0.1倍实际值（如角度100=10.0°，高度200=20.0mm）
 */
typedef struct {
    uint8_t  air_path_state;    // 气路状态：0=关闭，1=开启
    uint8_t  suck_state;        // 吸取状态：0=未吸到物料，1=已吸到物料
    int16_t  head_motor_angle;  // 头部电机角度（单位：0.1°，如数值1800=180.0°）
    int16_t  arm_servo_angles[11]; // 11路舵机/电机角度（单位：0.1°，数组索引0~10对应不同舵机）
    uint16_t lift_height;       // 升降机构高度（单位：0.1mm，如数值500=50.0mm）
} UpData_t;

// ===================== 下行数据结构体（PC→STM32） =====================
/**
 * @brief  下行数据结构体：PC发送给STM32的控制指令数据
 * @note   角度单位为弧度（float），高度单位为0.1mm（如数值500=50.0mm）
 */
typedef struct {
    float    pc_target_servo_angles[6]; // 6路舵机目标角度（单位：弧度，数组索引0~5对应不同舵机）
    float    pc_target_motor_angles[6]; // 6路电机目标角度（单位：弧度，数组索引0~5对应不同电机）
    uint8_t  pc_pump_state;        // 泵控制状态：0=关闭，1=开启
    uint16_t pc_target_lift_height;// 升降机构目标高度（单位：0.1mm，如数值500=50.0mm）
} DnData_t;

// ===================== 全局变量声明 =====================
extern DnData_t pc_dn_data;                // 存储PC下发的下行控制指令
extern uint8_t uart_protocol_raw_data[256];// 存储串口接收到的原始数据（最大256字节）

// ===================== 函数声明 =====================
/**
 * @brief  CRC16-CCITT校验算法
 * @param  data 待校验的数据缓冲区指针
 * @param  len  待校验数据的字节长度
 * @retval 计算得到的16位CRC校验值
 */
uint16_t crc16_ccitt(uint8_t *data, uint16_t len);

/**
 * @brief  封装上行数据帧（STM32→PC）
 * @param  data     待封装的上行数据结构体指针
 * @param  frame_buf 存储封装后完整帧数据的缓冲区指针
 * @retval 无
 */
void pack_up_frame(UpData_t *data, uint8_t *frame_buf);

/**
 * @brief  解析下行数据帧（PC→STM32）
 * @param  frame_buf 接收到的完整下行帧数据缓冲区指针
 * @param  data      存储解析后数据的结构体指针
 * @retval 无
 */
extern void unpack_dn_frame(uint8_t *frame_buf, DnData_t *data);

/**
 * @brief  发送封装好的串口帧数据
 * @param  huart    串口句柄指针（如&huart1、&huart2）
 * @param  frame_buf 待发送的帧数据缓冲区指针
 * @param  len      帧数据的总字节长度
 * @retval HAL状态枚举：HAL_OK=发送成功，其他值=发送失败
 */
HAL_StatusTypeDef send_frame(UART_HandleTypeDef *huart, uint8_t *frame_buf, uint16_t len);

/**
 * @brief  存储串口协议接收到的原始数据
 * @param  data 待存储的原始数据缓冲区指针
 * @param  size 待存储数据的字节长度
 * @retval 无
 * @note   数据超出256字节时截断，未填满部分清零
 */
extern void store_uart_protocol_data(const uint8_t *data, uint16_t size);

#endif // __UART_PROTOCOL_H