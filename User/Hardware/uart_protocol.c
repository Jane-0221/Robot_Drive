#include "uart_protocol.h"
#include <string.h>

// 全局变量：存储串口协议原始数据（最大256字节）
uint8_t uart_protocol_raw_data[256];
// 全局变量：存储从PC端下发的下行数据结构体
DnData_t pc_dn_data;

/**
 * @brief  CRC16-CCITT校验算法
 * @param  data 待校验的数据缓冲区
 * @param  len  待校验数据的字节长度
 * @retval 计算得到的16位CRC校验值
 * @note   初始值0xFFFF，多项式0x1021，标准CRC16-CCITT算法
 */
uint16_t crc16_ccitt(uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFF;  // CRC初始值
    for (uint16_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;  // 数据字节与CRC高8位异或
        for (uint8_t j = 0; j < 8; j++) // 逐位计算CRC
        {
            if (crc & 0x8000)           // 检查最高位是否为1
                crc = (crc << 1) ^ 0x1021; // 左移后异或多项式
            else
                crc <<= 1;              // 仅左移
        }
    }
    return crc; // 返回最终CRC值
}

/**
 * @brief  封装上行数据帧（STM32→PC）
 * @param  data     待封装的上行数据结构体指针
 * @param  frame_buf 存储封装后完整帧数据的缓冲区
 * @retval 无
 * @note   帧格式：帧头(2字节)+帧类型(1)+数据长度(1)+数据段+CRC16(2)+帧尾(2)
 */
void pack_up_frame(UpData_t *data, uint8_t *frame_buf)
{
    uint8_t idx = 0;  // 帧缓冲区索引，用于逐字节填充数据
    // 1. 填充帧头（2字节，FRAME_HEADER1/2为协议宏定义）
    frame_buf[idx++] = FRAME_HEADER1;
    frame_buf[idx++] = FRAME_HEADER2;
    // 2. 填充帧类型（1字节，上行帧类型标识）
    frame_buf[idx++] = UP_FRAME_TYPE;
    // 3. 填充数据段长度（1字节，上行数据固定长度）
    frame_buf[idx++] = UP_DATA_LEN;
    // 4. 填充具体上行数据段
    frame_buf[idx++] = data->air_path_state;  // 气路状态（1字节）
    frame_buf[idx++] = data->suck_state;      // 吸取状态（1字节）
    // 头部电机角度（2字节，小端存储：低字节在前，高字节在后）
    frame_buf[idx++] = (uint8_t)(data->head_motor_angle & 0xFF);
    frame_buf[idx++] = (uint8_t)((data->head_motor_angle >> 8) & 0xFF);
    // 11路舵机/电机角度（每路2字节，共22字节，小端存储）
    for (int i = 0; i < 11; i++)
    {
        frame_buf[idx++] = (uint8_t)(data->arm_servo_angles[i] & 0xFF);
        frame_buf[idx++] = (uint8_t)((data->arm_servo_angles[i] >> 8) & 0xFF);
    }
    // 升降高度（2字节，小端存储）
    frame_buf[idx++] = (uint8_t)(data->lift_height & 0xFF);
    frame_buf[idx++] = (uint8_t)((data->lift_height >> 8) & 0xFF);
    // 5. 计算并填充CRC16校验值（校验范围：帧类型+数据长度+数据段）
    // 注：2 = 帧类型(1字节) + 数据长度(1字节)，校验长度=2+数据段长度
    uint16_t crc = crc16_ccitt(&frame_buf[2], UP_DATA_LEN + 2);
    frame_buf[idx++] = (uint8_t)(crc & 0xFF);    // CRC低字节
    frame_buf[idx++] = (uint8_t)((crc >> 8) & 0xFF); // CRC高字节
    // 6. 填充帧尾（2字节，FRAME_TAIL1/2为协议宏定义）
    frame_buf[idx++] = FRAME_TAIL1;
    frame_buf[idx++] = FRAME_TAIL2;
}

/**
 * @brief  解析下行数据帧（PC→STM32）
 * @param  frame_buf 接收到的完整下行帧数据缓冲区
 * @param  data      存储解析后数据的结构体指针
 * @retval 无
 * @note   跳过帧头(2)+帧类型(1)+数据长度(1)，从第5字节（索引4）开始解析数据
 */
void unpack_dn_frame(uint8_t *frame_buf, DnData_t *data)
{
    uint8_t idx = 4; // 数据起始索引：跳过前4字节（帧头2+帧类型1+数据长度1）
    // 解析12路目标角度（每路2字节，小端存储，拼接为16位整型）
    for (int i = 0; i < 12; i++)
    {
        data->pc_target_angles[i] = (int16_t)(frame_buf[idx] | (frame_buf[idx + 1] << 8));
        idx += 2; // 索引偏移2字节，处理下一路角度
    }
    // 解析泵控制状态（1字节）
    data->pc_pump_state = frame_buf[idx++];
    // 解析升降机构目标高度（2字节，小端存储，值除以10为实际高度）
    data->pc_target_lift_height = (uint16_t)((frame_buf[idx] | (frame_buf[idx + 1] << 8))/10);
}

/**
 * @brief  发送封装好的串口帧数据
 * @param  huart    串口句柄（如&huart1）
 * @param  frame_buf 待发送的帧数据缓冲区
 * @param  len      帧数据的总字节长度
 * @retval HAL状态：HAL_OK表示发送成功，其他值表示失败
 * @note   超时时间设置为100ms，适用于常规串口通信场景
 */
HAL_StatusTypeDef send_frame(UART_HandleTypeDef *huart, uint8_t *frame_buf, uint16_t len)
{
    return HAL_UART_Transmit(huart, frame_buf, len, 100); // 调用HAL库串口发送函数
}

/**
 * @brief  存储串口协议接收到的原始数据
 * @param  data 待存储的原始数据缓冲区
 * @param  size 待存储数据的字节长度
 * @retval 无
 * @note   1. 最大存储256字节，超出部分截断；2. 未填满部分清零
 */
void store_uart_protocol_data(const uint8_t *data, uint16_t size)
{
    // 入参校验：空指针或长度为0直接返回
    if (data == NULL || size == 0)
    {
        return;
    }

    // 确定实际拷贝长度（不超过缓冲区最大256字节）
    uint16_t copy_size = (size > 256) ? 256 : size;

    // 拷贝数据到全局缓冲区（memcpy保证数据完整拷贝）
    memcpy(uart_protocol_raw_data, data, copy_size);

    // 若拷贝长度不足256字节，剩余部分清零，避免脏数据
    if (copy_size < 256)
    {
        memset(uart_protocol_raw_data + copy_size, 0, 256 - copy_size);
    }
}