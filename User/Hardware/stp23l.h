#ifndef STP23L_H
#define STP23L_H

#include "stdint.h"


/************************* STP-23L协议固定宏定义 *************************/
#define STP23L_HEADER        0xAAU        // 数据包起始符
#define STP23L_DEV_ADDR      0x00U        // 设备地址（保留位固定为0）
#define STP23L_PACK_LEN      195U         // 整包数据总字节数：10字节包头+184字节数据域+1字节校验
#define STP23L_HEAD_LEN      10U          // 包头长度：4AA+1地址+1命令+2偏移+2长度
#define STP23L_DATA_LEN      0x00B8U      // 获取测量数据命令的固定数据域长度
#define STP23L_POINT_NUM     12U          // 一帧固定12个测量点
#define STP23L_POINT_BYTES   15U          // 每个测量点15字节（2+2+4+1+4+2）
#define STP23L_TS_BYTES      4U           // 时间戳固定4字节
#define STP23L_RECV_TIMEOUT  50U          // 解析超时时间(ms)，超时报错复位

/************************* STP-23L命令码宏定义 *************************/
#define STP23L_CMD_GET_DIST  0x02U        // 获取测量数据
#define STP23L_CMD_RESET     0x0DU        // 系统复位
#define STP23L_CMD_STOP      0x0FU        // 停止数据传输
#define STP23L_CMD_ACK       0x10U        // 应答命令
#define STP23L_CMD_VERSION   0x14U        // 获取传感器信息

/************************* 测量点单数据结构体（严格匹配协议） *************************/
typedef struct
{
    int16_t  distance;   // 距离(mm) - 2B
    uint16_t noise;      // 环境噪声 - 2B
    uint32_t peak;       // 接收强度信息 - 4B
    uint8_t  confidence; // 置信度 - 1B
    uint32_t intg;       // 积分次数 - 4B
    int16_t  reftof;     // 温度表征值 - 2B
}STP23L_PointDef;

/************************* STP-23L全局数据结构体（存储解析后所有数据） *************************/
typedef struct
{
    uint8_t         parse_ok;    // 解析完成标志：1-解析成功，0-未解析/解析失败
    uint8_t         parse_err;   // 解析错误标志：1-超时/校验失败，0-正常
    STP23L_PointDef points[STP23L_POINT_NUM]; // 12个测量点数据
    uint32_t        timestamp;   // 数据包时间戳 - 4B
    uint32_t        recv_cnt;    // 成功接收数据包计数
    uint8_t         cmd_code;    // 当前解析到的命令码
}STP23L_DataDef;

/************************* 全局数据结构体声明（extern，供其他文件直接调用） *************************/
extern STP23L_DataDef stp23l_data;

/************************* 核心函数声明 *************************/
/**
 * @brief  STP-23L单字节解析函数（串口中断直接调用）
 * @param  byte: 串口中断接收到的单个字节
 * @retval 无
 */
void STP23L_ParseByte(uint8_t byte);

/**
 * @brief  STP-23L解析状态机复位（手动复位/错误复位）
 * @param  无
 * @retval 无
 */
void STP23L_Reset(void);

/**
 * @brief  清除解析完成标志（其他文件使用数据后调用）
 * @param  无
 * @retval 无
 */
static inline void STP23L_ClearOkFlag(void)
{
    stp23l_data.parse_ok = 0;
}

#endif // STP23L_H