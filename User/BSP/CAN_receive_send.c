/**
 * @file CAN_receive_send.c
 * @author Siri (lixirui2017@outlook.com)
 * @brief can bsp层发送与接受
 * @version 0.2
 * @date 2024-10-19
 *
 * @copyright Copyright (c) 2024
 *
 */
#include "can_receive_send.h"
#include "dm4310_drv.h"
#include "string.h"
#include "Robstride04.h"
#include "arm.h"
#include "stdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "remote_control.h"
#include "music.h"
#include "LED.h"
#include <cmsis_os2.h>
#include "iwdg.h"
#include "buzzer.h"
#include "fdcan.h"
#include "DrEmpower_can.h"  // 必须包含大然电机驱动头文件
/* USER CODE END Includes */
// 注意：过滤器配置已在 fdcan.c 中由 CubeMX 完成，此处不再重复配置
// 本文件只负责数据收发处理和用户层初始化

FDCAN_RxHeaderTypeDef RxHeader1; // 可能用于其他用途，保留
uint8_t g_Can1RxData[64];        // 保留但建议不再使用（改用局部变量）
float daibaoshan[3];
motor_measure_t motor_data[33];

extern uint8_t rx_buffer[8];
extern int8_t READ_FLAG;
extern uint16_t can_id;

extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;

/**
 * @brief 初始化can（用户层初始化）
 *        注意：过滤器、中断、启动已在 fdcan.c 中完成，本函数无需重复调用 HAL 函数。
 *        若需额外初始化（如变量清零），可在此添加。
 */
void can_init(void)
{
  // 所有 HAL 相关的配置已由 MX_FDCANx_Init 完成，此处不再重复
  // 可添加用户初始化代码，例如清零标志数组
}

// damiao 电机数据解析（保持不变）
void get_motor_measure(motor_measure_t *ptr, uint8_t data[])
{
  (ptr)->last_ecd = (ptr)->ecd;
  (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);
  (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);
  (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);
  (ptr)->temperate = (data)[6];
}

/**
 * @brief 发送标准帧数据
 */
uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_STANDARD_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;

  // 数据长度映射
  if (len <= 8)
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  else if (len == 12)
    TxHeader.DataLength = FDCAN_DLC_BYTES_12;
  else if (len == 16)
    TxHeader.DataLength = FDCAN_DLC_BYTES_16;
  else if (len == 20)
    TxHeader.DataLength = FDCAN_DLC_BYTES_20;
  else if (len == 24)
    TxHeader.DataLength = FDCAN_DLC_BYTES_24;
  else if (len == 48)
    TxHeader.DataLength = FDCAN_DLC_BYTES_48;
  else if (len == 64)
    TxHeader.DataLength = FDCAN_DLC_BYTES_64;
  else
    return 1; // 不支持的长度

  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;

  if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
    Error_Handler();
  }
  return 0;
}

/**
 * @brief 发送扩展帧数据
 */
uint8_t canx_send_ext_data(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint32_t len)
{
  FDCAN_TxHeaderTypeDef TxHeader;
  TxHeader.Identifier = id;
  TxHeader.IdType = FDCAN_EXTENDED_ID;
  TxHeader.TxFrameType = FDCAN_DATA_FRAME;

  // 数据长度映射（同标准帧）
  if (len <= 8)
    TxHeader.DataLength = FDCAN_DLC_BYTES_8;
  else if (len == 12)
    TxHeader.DataLength = FDCAN_DLC_BYTES_12;
  else if (len == 16)
    TxHeader.DataLength = FDCAN_DLC_BYTES_16;
  else if (len == 20)
    TxHeader.DataLength = FDCAN_DLC_BYTES_20;
  else if (len == 24)
    TxHeader.DataLength = FDCAN_DLC_BYTES_24;
  else if (len == 48)
    TxHeader.DataLength = FDCAN_DLC_BYTES_48;
  else if (len == 64)
    TxHeader.DataLength = FDCAN_DLC_BYTES_64;
  else
    return 1;

  TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
  TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
  TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  TxHeader.MessageMarker = 0x00;

  if (HAL_FDCAN_AddMessageToTxFifoQ(hcan, &TxHeader, data) != HAL_OK)
  {
    Error_Handler();
  }
  return 0;
}

/**
 * @brief 接收数据（旧函数，不再在回调中使用，保留兼容性）
 *        注意：此函数返回值无意义，建议不要使用
 */
uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan, uint32_t RXFIFO, FDCAN_RxHeaderTypeDef *fdcan_RxHeader, uint8_t *buf)
{
  if (HAL_FDCAN_GetRxMessage(hfdcan, RXFIFO, fdcan_RxHeader, buf) != HAL_OK)
    return 0;
  // 原返回 DataLength>>16 是错误的，改为返回 0 或 DLC 值
  return 0;
}

/**
 * @brief 电机数据处理（圈数累加）
 */
void process_motor_data(motor_measure_t *motor_data)
{
  if (motor_data->last_ecd > 7000 && motor_data->ecd < 1000)
    motor_data->ecd_cnt += ((ECD_MAX - motor_data->last_ecd) + motor_data->ecd);
  else if (motor_data->last_ecd < 1000 && motor_data->ecd > 7000)
    motor_data->ecd_cnt -= ((ECD_MAX - motor_data->ecd) + motor_data->last_ecd);
  else
    motor_data->ecd_cnt += (motor_data->ecd - motor_data->last_ecd);
}

/**
 * @brief CAN接收回调（FDCAN IT0 中断）
 *        核心修改：循环读取 FIFO 直到空，避免溢出
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
  FDCAN_RxHeaderTypeDef rx_header;
  uint8_t rx_data[8];

  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0)
    return;

  while (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
  {
    if (hfdcan->Instance == FDCAN1)
    {
      // FDCAN1 的处理（如有）
    }
    else if (hfdcan->Instance == FDCAN2)
    {
      if (rx_header.IdType == FDCAN_STANDARD_ID)
      {
        // 先处理已知的其他电机（如大疆）
        switch (rx_header.Identifier)
        {
        case 4:
          damiao_fbdata(&arm_motor[Motor4], rx_data);
          break;
        case 5:
          damiao_fbdata(&arm_motor[Motor5], rx_data);
          break;
        case 6:
          damiao_fbdata(&arm_motor[Motor6], rx_data);
          break;
        default:
          // --- 新增：大然电机处理 ---
          // 提取电机ID（ID的高5位）
          uint8_t motor_id = (rx_header.Identifier >> 5) & 0x3F;
          if (motor_id == 11) // 您使用的大然电机ID
          {
            // 将接收到的数据存入驱动库全局变量
       
          }
          else if (motor_id == 12)
          {
    
          }
          else if (motor_id == 13)
          {
     
          }

          // 若还有其他标准帧电机，可在此扩展
          break;
        }
      }
      else if (rx_header.IdType == FDCAN_EXTENDED_ID)
      {
        // 扩展帧处理（原RobStride等）
        uint8_t target_id = (uint8_t)((rx_header.Identifier >> 8) & 0xFF);
        if (target_id == 0x01)
        {
          RobStride_Motor_Analysis(&motor1, rx_data, rx_header.Identifier);
        }
        else if (target_id == 0x02)
        {
          RobStride_Motor_Analysis(&motor2, rx_data, rx_header.Identifier);
        }
        else if (target_id == 0x03)
        {
          RobStride_Motor_Analysis(&motor3, rx_data, rx_header.Identifier);
        }
      }
    }
  }
}

/**
 * @brief CAN错误处理回调，重新初始化并恢复过滤器（同时配置标准和扩展）
 */
void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan)
{
  // 停止、反初始化、重新初始化
  HAL_FDCAN_Stop(hfdcan);
  HAL_FDCAN_DeInit(hfdcan);
  HAL_FDCAN_Init(hfdcan);

  // 重新配置过滤器（标准+扩展）
  FDCAN_FilterTypeDef sFilter;
  // 标准帧过滤器（接收所有标准帧）
  sFilter.IdType = FDCAN_STANDARD_ID;
  sFilter.FilterIndex = 0;
  sFilter.FilterType = FDCAN_FILTER_MASK;
  sFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilter.FilterID1 = 0x000;
  sFilter.FilterID2 = 0x000;
  HAL_FDCAN_ConfigFilter(hfdcan, &sFilter);

  // 扩展帧过滤器（接收所有扩展帧）
  sFilter.IdType = FDCAN_EXTENDED_ID;
  sFilter.FilterIndex = 0; // 独立索引空间
  sFilter.FilterID1 = 0x00000000;
  sFilter.FilterID2 = 0x00000000;
  HAL_FDCAN_ConfigFilter(hfdcan, &sFilter);

  // 可选：配置全局过滤器为接受所有（若不想依赖过滤器，可设 FDCAN_ACCEPT_IN_RX_FIFO0）
  HAL_FDCAN_ConfigGlobalFilter(hfdcan, FDCAN_ACCEPT_IN_RX_FIFO0, FDCAN_ACCEPT_IN_RX_FIFO0,
                               FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

  // 重新激活通知并启动
  HAL_FDCAN_ActivateNotification(hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(hfdcan);
}