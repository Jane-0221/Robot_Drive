#include "pt_sensor.h"
#include <stddef.h>
#include <string.h>
#include <math.h>
#include "usart.h"
#include  "stdio.h"
/************************* 全局变量实例化 *************************/
PT_ParseDataDef pt_data = {0};
uint8_t pt_raw_buf[PT_BUF_MAX_LEN] = {0};
float g_pressure_value = 0.0f;

// 2. 核心解析函数（无返回值，传入buf和size，结果存全局变量）
/************************* 静态辅助函数：校验从机应答帧校验位 *************************/
// 手册固定校验规则：帧内除校验位外所有字节累加和取低8位 = 校验位
static uint8_t PT_CalcCrc(uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0;
    // 最后1字节是校验位，不累加
    for(uint16_t i = 0; i < len - 1; i++)
    {
        crc += buf[i];
    }
    return crc;
}

/************************* 查找从机应答帧头0xAA *************************/
uint16_t PT_FindSlaveHeader(uint8_t *buf, uint16_t size)
{
    for(uint16_t i = 0; i < size; i++)
    {
        if(buf[i] == PT_SLAVE_HEADER)
        {
            return i; // 返回帧头位置
        }
    }
    return 0xFFFF; // 未找到帧头
}

/************************* 存储串口原始数据 *************************/
void pt_store_raw_data(const uint8_t *data, uint16_t size)
{
    if(data == NULL || size == 0)
    {
        return;
    }

    // 限制缓存大小，防止溢出
    uint16_t copy_len = (size > PT_BUF_MAX_LEN) ? PT_BUF_MAX_LEN : size;
    memcpy(pt_raw_buf, data, copy_len);
    
    // 剩余空间清0
    if(copy_len < PT_BUF_MAX_LEN)
    {
        memset(pt_raw_buf + copy_len, 0, PT_BUF_MAX_LEN - copy_len);
    }
}

/************************* 解析状态复位 *************************/
void PT_Reset(void)
{
    pt_data.parse_ok = PT_PARSE_ERR;    // 初始未解析
    pt_data.parse_err = PT_PARSE_OK;    // 初始无错误
    pt_data.val.temp_raw = 0;
    pt_data.val.temp_act = 0.0f;
    pt_data.val.press_raw = 0;
    pt_data.val.press_act = 0.0f;
    pt_data.curr_cmd = 0;
    pt_data.recv_cnt = 0;
    memset(pt_raw_buf, 0, PT_BUF_MAX_LEN);
}

/************************* 解析温度应答帧 *************************/
void PT_ParseTempData(uint8_t *buf, uint16_t size)
{
    // 初始化解析状态为失败
    pt_data.parse_ok = PT_PARSE_ERR;
    pt_data.parse_err = PT_PARSE_ERR;

    if(buf == NULL || size < PT_FRAME_LEN_RSP_TEMP)
    {
        pt_data.parse_err = PT_PARSE_OK;
        return;
    }

    // 查找从机帧头0xAA
    uint16_t header_pos = PT_FindSlaveHeader(buf, size);
    if(header_pos == 0xFFFF)
    {
        pt_data.parse_err = PT_PARSE_OK;
        return;
    }

    // 帧头偏移后，校验帧完整性
    if((size - header_pos) < PT_FRAME_LEN_RSP_TEMP)
    {
        pt_data.parse_err = PT_PARSE_OK;
        return;
    }

    // 提取帧数据（偏移帧头位置）
    uint8_t *frame = buf + header_pos;
    uint8_t data_len = frame[1];    // 数据长度字段
    uint8_t data_type = frame[2];   // 数据类型码
    uint8_t crc_recv = frame[5];    // 接收的校验位
    uint8_t crc_calc = 0;           // 计算的校验位

    // 协议硬校验：数据长度0x06 + 数据类型0x0A
    if(data_len != 0x06 || data_type != PT_RSP_TEMP_TYPE)
    {
        pt_data.parse_err = PT_PARSE_OK;
        return;
    }

    // 计算校验位并验证
    crc_calc = PT_CalcCrc(frame, PT_FRAME_LEN_RSP_TEMP);
    if(crc_calc != crc_recv)
    {
        pt_data.parse_err = PT_PARSE_OK;
        return;
    }

    // 提取2字节温度原始值（小端序：低字节在前，高字节在后）
    pt_data.val.temp_raw = (frame[4] << 8) | frame[3];
    // 转换为实际温度(℃)
    pt_data.val.temp_act = (float)pt_data.val.temp_raw / PT_TEMP_SCALE;
    
    // 更新解析状态为成功
    pt_data.parse_ok = PT_PARSE_OK;
    pt_data.parse_err = PT_PARSE_ERR;
    pt_data.recv_cnt++;
    pt_data.curr_cmd = PT_CMD_READ_TEMP;
}

/************************* 解析压力应答帧（核心修正） *************************/
void PT_ParsePressData(uint8_t *buf, uint16_t size)
{
    // 初始化解析状态为失败
    pt_data.parse_ok = PT_PARSE_ERR;
    pt_data.parse_err = PT_PARSE_ERR;

    if(buf == NULL || size < PT_FRAME_LEN_RSP_PRESS)
    {
        pt_data.parse_err = PT_PARSE_OK;
        return;
    }

    // 查找从机帧头0xAA
    uint16_t header_pos = PT_FindSlaveHeader(buf, size);
    if(header_pos == 0xFFFF)
    {
        pt_data.parse_err = PT_PARSE_OK;
        return;
    }

    // 帧头偏移后，校验帧完整性
    if((size - header_pos) < PT_FRAME_LEN_RSP_PRESS)
    {
        pt_data.parse_err = PT_PARSE_OK;
        return;
    }

    // 提取帧数据（偏移帧头位置）
    uint8_t *frame = buf + header_pos;
    uint8_t data_len = frame[1];    // 数据长度字段
    uint8_t data_type = frame[2];   // 数据类型码
    uint8_t crc_recv = frame[7];    // 接收的校验位
    uint8_t crc_calc = 0;           // 计算的校验位

    // 协议硬校验：数据长度0x08 + 数据类型0x09
    if(data_len != 0x08 || data_type != PT_RSP_PRESS_TYPE)
    {
        pt_data.parse_err = PT_PARSE_OK;
        return;
    }

    // 计算校验位并验证
    crc_calc = PT_CalcCrc(frame, PT_FRAME_LEN_RSP_PRESS);
    if(crc_calc != crc_recv)
    {
        pt_data.parse_err = PT_PARSE_OK;
        return;
    }

    // 提取4字节压力原始值（小端序：低字节在前，高字节在后）
    pt_data.val.press_raw = (frame[6] << 24) | (frame[5] << 16) | (frame[4] << 8) | frame[3];
    // 转换为实际压力(kPa)
    pt_data.val.press_act = (float)pt_data.val.press_raw * PT_PRESS_RESOLUTION;
    
    // 更新解析状态为成功
    pt_data.parse_ok = PT_PARSE_OK;
    pt_data.parse_err = PT_PARSE_ERR;
    pt_data.recv_cnt++;
    pt_data.curr_cmd = PT_CMD_READ_PRESS;
}

/************************* 获取实际温度值 *************************/
float PT_GetActTemp(void)
{
    if(pt_data.parse_ok == PT_PARSE_OK && pt_data.curr_cmd == PT_CMD_READ_TEMP)
    {
        return pt_data.val.temp_act;
    }
    return 0.0f; // 解析失败返回0
}

/************************* 获取实际压力值 *************************/
float PT_GetActPress(void)
{
    if(pt_data.parse_ok == PT_PARSE_OK && pt_data.curr_cmd == PT_CMD_READ_PRESS)
    {
        return pt_data.val.press_act;
    }
    return 0.0f; // 解析失败返回0
}

/**
 * @brief  发送读取温度指令（HAL库）
 * @param  huart: 串口句柄指针（如&huart1、&huart2）
 * @retval 无
 * @note   指令帧：55 04 0E 6A，超时时间100ms
 */
void PT_Send_ReadTemp_Cmd(UART_HandleTypeDef *huart)
{
    // 空指针校验，避免传入NULL导致程序崩溃
    if(huart == NULL)
    {
        return;
    }
    
    // 读取温度的指令帧（来自pt_sensor.h的宏定义）
    uint8_t temp_cmd[] = PT_CMD_FRAME_TEMP;
    
    // 发送4字节指令（HAL库阻塞式发送，超时100ms）
    HAL_UART_Transmit(huart, temp_cmd, sizeof(temp_cmd), 100);
}

/**
 * @brief  发送读取压力指令（HAL库）
 * @param  huart: 串口句柄指针（如&huart1、&huart2）
 * @retval 无
 * @note   指令帧：55 04 0D 88，超时时间100ms；需先发送温度指令再调用此函数
 */
void PT_Send_ReadPress_Cmd(UART_HandleTypeDef *huart)
{
    // 空指针校验
    if(huart == NULL)
    {
        return;
    }
    
    // 读取压力的指令帧（来自pt_sensor.h的宏定义）
    uint8_t press_cmd[] = PT_CMD_FRAME_PRESS;
    
    // 发送4字节指令
    HAL_UART_Transmit(huart, press_cmd, sizeof(press_cmd), 100);
}

/**
 * @brief  解析混合在自定义缓存中的温度帧和压力帧（支持传入pt_raw_buf[256]）
 * @param  buf: 传入的原始数据缓存（如uint8_t pt_raw_buf[256]）
 * @param  buf_len: buf中有效数据的长度（串口实际接收的字节数，不是256）
 * @retval 无
 * @note   1. 自动识别AA 06 0A（温度帧头）和AA 08 09（压力帧头）
 *         2. 提取完整帧后调用PT_ParseTempData/PT_ParsePressData解析
 *         3. 兼容多帧混合、帧重叠，内置越界保护
 */
void PT_ParseMixedFrames(uint8_t *buf, uint16_t buf_len)
{


    // 空指针/无效长度校验
    if(buf == NULL || buf_len == 0 || buf_len > PT_BUF_MAX_LEN)
    {
        return;
    }

    uint16_t i = 0;
    // 遍历缓存，逐个查找帧头（AA）
    while(i < buf_len)
    {
        // 步骤1：定位帧头0xAA
        if(buf[i] != PT_SLAVE_HEADER)
        {
            i++;
            continue;
        }

        // 步骤2：验证剩余长度是否够验证帧类型（至少3字节：AA 06 0A / AA 08 09）
        if((i + 3) > buf_len)
        {
            break; // 剩余数据不足，退出遍历
        }

        // 步骤3：识别帧类型，确定帧总长度
        uint8_t frame_type = 0; // 0-未知，1-温度帧，2-压力帧
        uint16_t frame_total_len = 0;

        // 温度帧特征：AA 06 0A，完整帧6字节
        if(buf[i+1] == 0x06 && buf[i+2] == 0x0A)
        {
            frame_type = 1;
            frame_total_len = PT_FRAME_LEN_RSP_TEMP; // 6字节
        }
        // 压力帧特征：AA 08 09，完整帧8字节
        else if(buf[i+1] == 0x08 && buf[i+2] == 0x09)
        {
            frame_type = 2;
            frame_total_len = PT_FRAME_LEN_RSP_PRESS; // 8字节
        }

        // 步骤4：验证帧完整性（剩余长度是否够完整帧）
        if((i + frame_total_len) > buf_len || frame_type == 0)
        {
            i++;
            continue;
        }

        // 步骤5：直接解析完整帧（无需临时数组，减少内存拷贝）
        if(frame_type == 1) // 处理温度帧
        {
            printf("温度帧: ");
            for(uint16_t j = 0; j < frame_total_len; j++)
            {
                printf("%02X ", buf[i + j]);
            }
            printf("\n");
            PT_ParseTempData(&buf[i], frame_total_len);
        }
        else if(frame_type == 2) // 处理压力帧
        {
            printf("压力帧: ");
            for(uint16_t j = 0; j < frame_total_len; j++)
            {
                printf("%02X ", buf[i + j]);
            }
            printf("\n");
            PT_ParsePressData(&buf[i], frame_total_len);
        }

        // 步骤6：跳过已处理的帧，继续遍历剩余数据
        i += frame_total_len;
    }
}
// 1. 全局气压值变量（放在.c文件全局区域，.h文件声明 extern float g_pressure_value;）

void PT_ParsePressureToGlobal(uint8_t *buf, uint16_t size)
{
    // 初始化全局变量为0（解析失败时保持0.0f）
    g_pressure_value = 0.0f;

    // 参数校验：空指针/无效长度直接返回
    if(buf == NULL || size == 0 || size > 256)
    {
        return;
    }

    uint16_t i = 0;
    // 遍历缓存查找压力帧（AA 08 09开头）
    while(i < size)
    {
        // 定位帧头0xAA
        if(buf[i] != 0xAA)
        {
            i++;
            continue;
        }

        // 验证剩余长度够识别压力帧（至少3字节）
        if((i + 3) > size)
        {
            break;
        }

        // 识别压力帧特征：AA 08 09
        if(buf[i+1] == 0x08 && buf[i+2] == 0x09)
        {
            // 验证帧完整性（完整压力帧8字节）
            if((i + 8) > size)
            {
                i++;
                continue;
            }

            // 提取4字节压力原始值（小端序转大端）
            uint8_t *frame = &buf[i];
            uint32_t press_raw = (frame[6] << 24) | (frame[5] << 16) | (frame[4] << 8) | frame[3];
            
            // 换算为实际气压值（×0.001kPa），存入全局变量
            g_pressure_value = (float)press_raw * 0.001f;
        }

        i++;
    }
}