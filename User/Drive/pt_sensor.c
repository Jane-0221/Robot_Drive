#include "pt_sensor.h"
#include <stddef.h>
#include <string.h>
#include <math.h>

/************************* 全局变量实例化（与头文件声明对应） *************************/
PT_ParseDataDef pt_data = {0};
uint8_t pt_raw_buf[PT_BUF_MAX_LEN] = {0};

/************************* 静态辅助函数：校验从机应答帧校验位（累加和低8位） *************************/
// 手册固定校验规则：帧内除校验位外所有字节累加和取低8位 = 校验位
static uint8_t PT_CalcCrc(uint8_t *buf, uint16_t len)
{
    uint8_t crc = 0;
    for(uint16_t i = 0; i < len - 1; i++) // 最后1字节是校验位，不累加
    {
        crc += buf[i];
    }
    return crc;
}

/************************* 查找从机应答帧头0xAA（参考STP23L_FindHeader） *************************/
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

/************************* 存储串口原始数据（参考store_stp23l_data） *************************/
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

/************************* 解析状态复位（参考STP23L_Reset） *************************/
void PT_Reset(void)
{
    pt_data.parse_ok = PT_PARSE_ERR;
    pt_data.parse_err = PT_PARSE_ERR;
    pt_data.val.temp_raw = 0;
    pt_data.val.temp_act = 0.0f;
    pt_data.val.press_raw = 0;
    pt_data.val.press_act = 0.0f;
    pt_data.curr_cmd = 0;
    // 保留成功接收计数，如需清计数可在此处加 pt_data.recv_cnt = 0;
    memset(pt_raw_buf, 0, PT_BUF_MAX_LEN);
}

/************************* 解析温度应答帧（0xAA 0x06 0x0A 2字节温度 校验位） *************************/
void PT_ParseTempData(uint8_t *buf, uint16_t size)
{
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
    // 提取2字节温度原始值（手册：0x0102=258 → 25.8℃，大端模式）
    pt_data.val.temp_raw = (frame[3] << 8) | frame[4];
    // 转换为实际温度(℃)
    pt_data.val.temp_act = (float)pt_data.val.temp_raw / PT_TEMP_SCALE;
    // 更新解析状态
    pt_data.parse_ok = PT_PARSE_OK;
    pt_data.parse_err = PT_PARSE_ERR;
    pt_data.recv_cnt++;
    pt_data.curr_cmd = PT_CMD_READ_TEMP;
}

/************************* 解析压力应答帧（0xAA 0x08 0x09 4字节压力 校验位） *************************/
void PT_ParsePressData(uint8_t *buf, uint16_t size)
{
    if(buf == NULL || size < PT_FRAME_LEN_RSP_PRESS || pt_data.val.temp_act == 0.0f)
    {
        // 协议要求：先读温度再读压力，未解析温度则直接报错
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
    // 提取4字节压力原始值（手册：0xA0860100 → 0x000186A0=100kPa，逆序转大端）
    pt_data.val.press_raw = (frame[6] << 24) | (frame[5] << 16) | (frame[4] << 8) | frame[3];
    // 转换为实际压力(kPa)（手册固定数值映射规则）
    pt_data.val.press_act = (float)pt_data.val.press_raw * PT_PRESS_RESOLUTION;
    // 更新解析状态
    pt_data.parse_ok = PT_PARSE_OK;
    pt_data.parse_err = PT_PARSE_ERR;
    pt_data.recv_cnt++;
    pt_data.curr_cmd = PT_CMD_READ_PRESS;
}

/************************* 获取实际温度值（外部调用，返回有效温度） *************************/
float PT_GetActTemp(void)
{
    if(pt_data.parse_ok == PT_PARSE_OK && pt_data.curr_cmd == PT_CMD_READ_TEMP)
    {
        return pt_data.val.temp_act;
    }
    return 0.0f; // 解析失败返回0
}

/************************* 获取实际压力值（外部调用，返回有效压力） *************************/
float PT_GetActPress(void)
{
    if(pt_data.parse_ok == PT_PARSE_OK && pt_data.curr_cmd == PT_CMD_READ_PRESS)
    {
        return pt_data.val.press_act;
    }
    return 0.0f; // 解析失败返回0
}