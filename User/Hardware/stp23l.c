#include "stp23l.h"

/************************* 全局数据结构体实例化（供extern调用） *************************/
STP23L_DataDef stp23l_data = {0};

/************************* 解析状态机静态变量（仅模块内可见，无全局污染） *************************/
static uint8_t  stp23l_state = 0;    // 解析状态机状态
static uint8_t  stp23l_crc = 0;      // 校验和（自动低8位溢出，匹配协议）
static uint16_t stp23l_datalen = 0;  // 协议数据长度域
static uint16_t stp23l_offset = 0;   // 分块偏移地址
static uint8_t  stp23l_pnt_cnt = 0;  // 测量点计数（0~11）
static uint32_t stp23l_last_tick = 0;// 最后接收字节的系统时间（超时判断）

/************************* 状态机复位实现 *************************/
void STP23L_Reset(void)
{
    // 复位状态机变量
    stp23l_state = 0;
    stp23l_crc = 0;
    stp23l_datalen = 0;
    stp23l_offset = 0;
    stp23l_pnt_cnt = 0;
    stp23l_last_tick = HAL_GetTick();
    // 复位错误标志
    stp23l_data.parse_err = 0;
}

/************************* 核心单字节解析实现 *************************/
void STP23L_ParseByte(uint8_t byte)
{
    uint8_t point_byte_idx = 0;  // 测量点内字节偏移（0~14）
    uint8_t ts_byte_idx = 0;     // 时间戳内字节偏移（0~3）

    /************************* 第一步：超时判断，超时则复位状态机 *************************/
    if((HAL_GetTick() - stp23l_last_tick) > STP23L_RECV_TIMEOUT)
    {
        stp23l_data.parse_err = 1;
        STP23L_Reset();
        return;
    }
    stp23l_last_tick = HAL_GetTick(); // 更新最后接收时间

    /************************* 第二步：解析4个起始符0xAA *************************/
    if(stp23l_state < 4)
    {
        if(byte == STP23L_HEADER)
        {
            stp23l_state++;
        }
        else
        {
            stp23l_data.parse_err = 1;
            STP23L_Reset(); // 非起始符，直接复位
        }
        return;
    }

    /************************* 第三步：解析包头（设备地址+命令码+偏移+数据长度） *************************/
    if((stp23l_state >= 4) && (stp23l_state < STP23L_HEAD_LEN))
    {
        switch(stp23l_state)
        {
            case 4: // 解析设备地址
                if(byte == STP23L_DEV_ADDR)
                {
                    stp23l_crc += byte; // 校验和从设备地址开始累加（协议要求除去4个AA）
                    stp23l_state++;
                }
                else
                {
                    stp23l_data.parse_err = 1;
                    STP23L_Reset();
                }
                break;
            case 5: // 解析命令码，仅处理测量数据命令
                if(byte == STP23L_CMD_GET_DIST)
                {
                    stp23l_data.cmd_code = byte;
                    stp23l_crc += byte;
                    stp23l_state++;
                }
                else
                {
                    stp23l_data.parse_err = 1;
                    STP23L_Reset(); // 非测量数据命令，直接复位（可根据需求扩展）
                }
                break;
            case 6: // 解析分块偏移地址低字节
                stp23l_offset = byte;
                stp23l_crc += byte;
                stp23l_state++;
                break;
            case 7: // 解析分块偏移地址高字节
                stp23l_offset |= (uint16_t)byte << 8;
                stp23l_crc += byte;
                stp23l_state++;
                break;
            case 8: // 解析数据长度低字节
                stp23l_datalen = byte;
                stp23l_crc += byte;
                stp23l_state++;
                break;
            case 9: // 解析数据长度高字节+校验数据长度
                stp23l_datalen |= (uint16_t)byte << 8;
                stp23l_crc += byte;
                stp23l_state++;
                // 协议强制校验：测量数据命令的长度必须为0x00B8，否则丢弃
                if(stp23l_datalen != STP23L_DATA_LEN)
                {
                    stp23l_data.parse_err = 1;
                    STP23L_Reset();
                }
                break;
            default:
                stp23l_data.parse_err = 1;
                STP23L_Reset();
                break;
        }
        return;
    }

    /************************* 第四步：解析数据域（12个测量点+4字节时间戳） *************************/
    if((stp23l_state >= STP23L_HEAD_LEN) && (stp23l_state < STP23L_PACK_LEN - 1))
    {
        // 解析12个测量点（0~179字节：STP23L_HEAD_LEN ~ STP23L_HEAD_LEN+12*15-1）
        if(stp23l_state < (STP23L_HEAD_LEN + STP23L_POINT_NUM * STP23L_POINT_BYTES))
        {
            // 计算当前测量点内的字节偏移（0~14）
            point_byte_idx = (stp23l_state - STP23L_HEAD_LEN) % STP23L_POINT_BYTES;
            // 小端模式逐字节解析测量点数据
            switch(point_byte_idx)
            {
                case 0: stp23l_data.points[stp23l_pnt_cnt].distance = byte; break;
                case 1: stp23l_data.points[stp23l_pnt_cnt].distance |= (int16_t)byte << 8; break;
                case 2: stp23l_data.points[stp23l_pnt_cnt].noise = byte; break;
                case 3: stp23l_data.points[stp23l_pnt_cnt].noise |= (uint16_t)byte << 8; break;
                case 4: stp23l_data.points[stp23l_pnt_cnt].peak = byte; break;
                case 5: stp23l_data.points[stp23l_pnt_cnt].peak |= (uint32_t)byte << 8; break;
                case 6: stp23l_data.points[stp23l_pnt_cnt].peak |= (uint32_t)byte << 16; break;
                case 7: stp23l_data.points[stp23l_pnt_cnt].peak |= (uint32_t)byte << 24; break;
                case 8: stp23l_data.points[stp23l_pnt_cnt].confidence = byte; break;
                case 9: stp23l_data.points[stp23l_pnt_cnt].intg = byte; break;
                case 10: stp23l_data.points[stp23l_pnt_cnt].intg |= (uint32_t)byte << 8; break;
                case 11: stp23l_data.points[stp23l_pnt_cnt].intg |= (uint32_t)byte << 16; break;
                case 12: stp23l_data.points[stp23l_pnt_cnt].intg |= (uint32_t)byte << 24; break;
                case 13: stp23l_data.points[stp23l_pnt_cnt].reftof = byte; break;
                case 14: // 单个测量点解析完成，计数+1
                    stp23l_data.points[stp23l_pnt_cnt].reftof |= (int16_t)byte << 8;
                    stp23l_pnt_cnt++;
                    break;
                default: break;
            }
            stp23l_crc += byte;
            stp23l_state++;
            return;
        }

        // 解析4字节时间戳（180~183字节：STP23L_HEAD_LEN+12*15 ~ STP23L_HEAD_LEN+183）
        if(stp23l_state < (STP23L_HEAD_LEN + STP23L_DATA_LEN))
        {
            ts_byte_idx = stp23l_state - (STP23L_HEAD_LEN + STP23L_POINT_NUM * STP23L_POINT_BYTES);
            switch(ts_byte_idx)
            {
                case 0: stp23l_data.timestamp = byte; break;
                case 1: stp23l_data.timestamp |= (uint32_t)byte << 8; break;
                case 2: stp23l_data.timestamp |= (uint32_t)byte << 16; break;
                case 3: stp23l_data.timestamp |= (uint32_t)byte << 24; break;
                default: break;
            }
            stp23l_crc += byte;
            stp23l_state++;
            return;
        }
    }

    /************************* 第五步：解析校验码并验证，完成解析 *************************/
    if(stp23l_state == STP23L_PACK_LEN - 1)
    {
        // 校验码匹配：接收的字节 == 累加的校验和（低8位）
        if(byte == stp23l_crc)
        {
            stp23l_data.parse_ok = 1;    // 置位解析完成标志
            stp23l_data.recv_cnt++;      // 成功接收计数+1
            stp23l_data.parse_err = 0;   // 清除错误标志
        }
        else
        {
            stp23l_data.parse_err = 1;   // 校验失败，置位错误标志
        }
        STP23L_Reset(); // 解析完成/失败，均复位状态机，准备下一包解析
    }
}