#ifndef PT_SENSOR_H
#define PT_SENSOR_H
#include "stdint.h"

/************************* 协议核心宏定义（严格遵循传感器手册） *************************/
// 帧头定义：主机指令帧头0x55，从机应答帧头0xAA
#define PT_HOST_HEADER       0x55U        // 主机发送指令帧头
#define PT_SLAVE_HEADER      0xAAU        // 从机应答数据帧头
#define PT_HEADER_BYTES      1U           // 单字节帧头
// 串口配置（与手册一致）
#define PT_BAUDRATE          9600U        // 波特率9600
#define PT_DATA_BITS         8U           // 数据位8
#define PT_STOP_BITS         1U           // 停止位1
#define PT_PARITY_NONE       0U           // 无校验位
// 指令码：主机控制指令/从机数据类型码
#define PT_CMD_READ_TEMP     0x0EU        // 主机读温度控制指令
#define PT_CMD_READ_PRESS    0x0DU        // 主机读压力控制指令
#define PT_CMD_SLEEP         0x01U        // 主机进入休眠控制指令
#define PT_RSP_TEMP_TYPE     0x0AU        // 从机温度应答数据类型码
#define PT_RSP_PRESS_TYPE    0x09U        // 从机压力应答数据类型码
#define PT_RSP_SLEEP_TYPE    0x01U        // 从机休眠应答类型码
// 帧长度定义（手册固定帧格式）
#define PT_FRAME_LEN_HOST    4U           // 主机指令帧总长度：帧头+数据长度+指令+校验（4字节）
#define PT_FRAME_LEN_RSP_TEMP 6U          // 从机温度应答帧长度：帧头+数据长度+类型+2字节温度+校验（6字节）
#define PT_FRAME_LEN_RSP_PRESS 8U         // 从机压力应答帧长度：帧头+数据长度+类型+4字节压力+校验（8字节）
#define PT_FRAME_LEN_RSP_SLEEP 4U         // 从机休眠应答帧长度：帧头+数据长度+类型+校验（4字节）
#define PT_DATA_LEN_DEF      0x04U        // 主机指令帧固定数据长度字段0x04
// 数据解析宏
#define PT_TEMP_SCALE        10.0f        // 温度原始值转换系数（除以10得实际温度）
#define PT_PRESS_RESOLUTION  0.01f        // 压力分辨率0.01kPa（手册定义）
// 解析状态宏
#define PT_PARSE_OK          1U
#define PT_PARSE_ERR         0U
#define PT_BUF_MAX_LEN       256U         // 原始数据缓存最大长度（与stp23l一致）

/************************* 温压传感器数据结构体（贴合手册解析规则） *************************/
// 温压原始+实际数据结构体
typedef struct
{
    uint16_t temp_raw;    // 温度原始值（2字节，从机应答）
    float temp_act;       // 实际温度(℃) = temp_raw / PT_TEMP_SCALE
    uint32_t press_raw;   // 压力原始值（4字节，从机应答）
    float press_act;      // 实际压力(kPa)（按手册十六进制转十进制规则）
}PT_SensorDataDef;

// 全局解析状态+数据结构体（extern供外部调用，参考stp23l）
typedef struct
{
    uint8_t parse_ok;     // 解析完成标志：1-解析成功，0-未解析/解析失败
    uint8_t parse_err;    // 解析错误标志：1-帧错误/校验错误，0-正常
    PT_SensorDataDef val; // 温压原始+实际值
    uint32_t recv_cnt;    // 成功接收应答帧数（温度+压力分别计数）
    uint8_t curr_cmd;     // 当前主机发送的指令码
}PT_ParseDataDef;

/************************* 全局变量声明（extern） *************************/
extern PT_ParseDataDef pt_data;
extern uint8_t pt_raw_buf[PT_BUF_MAX_LEN]; // 原始串口数据缓存

/************************* 核心函数声明（参考stp23l功能设计） *************************/
void PT_Reset(void);                          // 解析状态复位（清标志/计数）
void pt_store_raw_data(const uint8_t *data, uint16_t size); // 存储串口原始数据
uint16_t PT_FindSlaveHeader(uint8_t *buf, uint16_t size);   // 查找从机应答帧头0xAA
void PT_ParseTempData(uint8_t *buf, uint16_t size);         // 解析温度应答帧
void PT_ParsePressData(uint8_t *buf, uint16_t size);        // 解析压力应答帧
float PT_GetActTemp(void);                                 // 获取实际温度值
float PT_GetActPress(void);                                // 获取实际压力值
static inline void PT_ClearParseOk(void)                   // 清除解析完成标志（内联函数）
{
    pt_data.parse_ok = PT_PARSE_ERR;
}

#endif // PT_SENSOR_H