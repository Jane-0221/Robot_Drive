#include "uart_protocol.h"
#include <string.h>
uint8_t uart_protocol_raw_data[256];
DnData_t pc_dn_data;
// CRC16-CCITT校验
uint16_t crc16_ccitt(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 0x8000)
                crc = (crc << 1) ^ 0x1021;
            else
                crc <<= 1;
        }
    }
    return crc;
}

// 打包上行帧（STM32→主机）
void pack_up_frame(UpData_t *data, uint8_t *frame_buf)
{
    uint8_t idx = 0;
    // 1. 帧头
    frame_buf[idx++] = FRAME_HEADER1;
    frame_buf[idx++] = FRAME_HEADER2;
    // 2. 帧类型
    frame_buf[idx++] = UP_FRAME_TYPE;
    // 3. 数据长度
    frame_buf[idx++] = UP_DATA_LEN;
    // 4. 数据域
    frame_buf[idx++] = data->air_path_state;
    frame_buf[idx++] = data->suck_state;
    // 头部电机角度（小端）
    frame_buf[idx++] = (uint8_t)(data->head_motor_angle & 0xFF);
    frame_buf[idx++] = (uint8_t)((data->head_motor_angle >> 8) & 0xFF);
    // 11路机械臂/舵机角度
    for (int i = 0; i < 11; i++)
    {
        frame_buf[idx++] = (uint8_t)(data->arm_servo_angles[i] & 0xFF);
        frame_buf[idx++] = (uint8_t)((data->arm_servo_angles[i] >> 8) & 0xFF);
    }
    // 升降杆高度
    frame_buf[idx++] = (uint8_t)(data->lift_height & 0xFF);
    frame_buf[idx++] = (uint8_t)((data->lift_height >> 8) & 0xFF);
    // 5. CRC16校验（帧类型+数据长度+数据域）
    uint16_t crc = crc16_ccitt(&frame_buf[2], UP_DATA_LEN + 2); // 2=帧类型+数据长度
    frame_buf[idx++] = (uint8_t)(crc & 0xFF);
    frame_buf[idx++] = (uint8_t)((crc >> 8) & 0xFF);
    // 6. 帧尾
    frame_buf[idx++] = FRAME_TAIL1;
    frame_buf[idx++] = FRAME_TAIL2;
}

// 解析下行帧（主机→STM32）
void unpack_dn_frame(uint8_t *frame_buf, DnData_t *data)
{
    uint8_t idx = 4; // 跳过帧头(2)+帧类型(1)+数据长度(1)
    // 12路目标角度
    for (int i = 0; i < 12; i++)
    {
        data->pc_target_angles[i] = (int16_t)(frame_buf[idx] | (frame_buf[idx + 1] << 8));
        idx += 2;
    }
    // 气泵状态
    data->pc_pump_state = frame_buf[idx++];
    // 升降杆目标高度
    data->pc_target_lift_height = (uint16_t)(frame_buf[idx] | (frame_buf[idx + 1] << 8));
}

// 发送帧（带超时）
HAL_StatusTypeDef send_frame(UART_HandleTypeDef *huart, uint8_t *frame_buf, uint16_t len)
{
    return HAL_UART_Transmit(huart, frame_buf, len, 100); // 超时100ms
}
void store_uart_protocol_data(const uint8_t *data, uint16_t size)
{
    if (data == NULL || size == 0)
    {
        return;
    }

    // 确保不会超出缓冲区大小
    uint16_t copy_size = (size > 256) ? 256 : size;

    // 使用memcpy安全地复制数据
    memcpy(uart_protocol_raw_data, data, copy_size);

    // 如果数据小于256字节，剩余部分填充0
    if (copy_size < 256)
    {
        memset(uart_protocol_raw_data + copy_size, 0, 256 - copy_size);
    }
}