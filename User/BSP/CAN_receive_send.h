#ifndef __CAN_RECEIVE_SEND_H__
#define __CAN_RECEIVE_SEND_H__
#include "fdcan.h"



typedef FDCAN_HandleTypeDef hcan_t;//
 

extern void can_init(void);
extern uint8_t canx_send_data(FDCAN_HandleTypeDef *hcan, uint16_t id, uint8_t *data, uint32_t len);
extern uint8_t canx_send_ext_data(FDCAN_HandleTypeDef *hcan, uint32_t id, uint8_t *data, uint32_t len);
extern uint8_t fdcanx_receive(FDCAN_HandleTypeDef *hfdcan,uint32_t RXFIFO,FDCAN_RxHeaderTypeDef *fdcan_RxHeader,uint8_t *buf);
extern void CAN1_send_current(void); // 发送电机控制电流
extern void CAN2_send_current(void); // 发送电机控制电流
extern void CAN3_send_current(void); // 发送电机控制电流
extern void HAL_FDCAN_ErrorCallback(FDCAN_HandleTypeDef *hfdcan);


#endif /* __CAN_RECEIVE_SEND_H__ */






