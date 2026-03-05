#ifndef __UART_PROTOCOL_H
#define __UART_PROTOCOL_H

#include "stm32h7xx_hal.h"

// 帧格式定义
#define FRAME_HEADER1 0xAA
#define FRAME_HEADER2 0x55
#define FRAME_TAIL1   0xEE
#define FRAME_TAIL2   0xFF
#define UP_FRAME_TYPE 0x01  // 上行帧
#define DN_FRAME_TYPE 0x02  // 下行帧

// 数据域长度
#define UP_DATA_LEN   28    // 上行数据域长度
#define DN_DATA_LEN   27    // 下行数据域长度

// 上行数据结构体（STM32→主机）
typedef struct {
    uint8_t  air_path_state;    // 气路状态：0=关，1=开
    uint8_t  suck_state;        // 吸附状态：0=未吸住，1=吸住
    int16_t  head_motor_angle;  // 头部电机角度（0.1°）
    int16_t  arm_servo_angles[11]; // 11路机械臂/舵机角度（0.1°）
    uint16_t lift_height;       // 升降杆高度（0.1mm）
} UpData_t;

// 下行数据结构体（主机→STM32）
typedef struct {
    int16_t  pc_target_angles[12]; // 12路电机/舵机目标角度（0.1°）
    uint8_t  pc_pump_state;        // 气泵状态：0=关，1=开
    uint16_t pc_target_lift_height;// 升降杆目标高度（0.1mm）
} DnData_t;
extern DnData_t pc_dn_data;
extern uint8_t uart_protocol_raw_data[256];
// 函数声明
uint16_t crc16_ccitt(uint8_t *data, uint16_t len);
void pack_up_frame(UpData_t *data, uint8_t *frame_buf);  // 打包上行帧
extern void unpack_dn_frame(uint8_t *frame_buf, DnData_t *data); // 解析下行帧
HAL_StatusTypeDef send_frame(UART_HandleTypeDef *huart, uint8_t *frame_buf, uint16_t len);
extern void store_uart_protocol_data(const uint8_t *data, uint16_t size);
#endif