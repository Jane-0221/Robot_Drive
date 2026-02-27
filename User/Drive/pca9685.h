#ifndef PCA9685_H
#define PCA9685_H

#include <stdint.h>
#include "stm32h7xx_hal.h"  // 根据所用芯片修改，或包含你的HAL头文件

// 寄存器地址
#define PCA9685_MODE1         0x00
#define PCA9685_PRESCALE      0xFE

#define LED0_ON_L             0x06
#define LED0_ON_H             0x07
#define LED0_OFF_L            0x08
#define LED0_OFF_H            0x09

#define ALLLED_ON_L           0xFA
#define ALLLED_ON_H           0xFB
#define ALLLED_OFF_L          0xFC
#define ALLLED_OFF_H          0xFD

// 默认I2C地址
#define PCA9685_I2C_ADDR      0x40

// 手柄结构体
typedef struct {
    I2C_HandleTypeDef *hi2c;   // I2C句柄
    uint8_t addr;               // 设备地址
} PCA9685_HandleTypeDef;

// 初始化
void PCA9685_Init(PCA9685_HandleTypeDef *hpca, I2C_HandleTypeDef *hi2c, uint8_t addr);

// 复位
void PCA9685_Reset(PCA9685_HandleTypeDef *hpca);

// 设置PWM频率
void PCA9685_SetPWMFreq(PCA9685_HandleTypeDef *hpca, float freq);

// 设置单个通道PWM（on和off为12位计数值）
void PCA9685_SetPWM(PCA9685_HandleTypeDef *hpca, uint8_t num, uint16_t on, uint16_t off);

// 简化设置：val 0~4095，invert=1时反转极性
void PCA9685_SetPin(PCA9685_HandleTypeDef *hpca, uint8_t num, uint16_t val, uint8_t invert);

#endif