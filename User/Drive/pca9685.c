#include "pca9685.h"
#include <math.h>   // 用于floor

// 内部读写函数
static HAL_StatusTypeDef PCA9685_Write8(PCA9685_HandleTypeDef *hpca, uint8_t reg, uint8_t data)
{
    return HAL_I2C_Mem_Write(hpca->hi2c, hpca->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, HAL_MAX_DELAY);
}

static HAL_StatusTypeDef PCA9685_Read8(PCA9685_HandleTypeDef *hpca, uint8_t reg, uint8_t *data)
{
    return HAL_I2C_Mem_Read(hpca->hi2c, hpca->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 1, HAL_MAX_DELAY);
}

// 初始化
void PCA9685_Init(PCA9685_HandleTypeDef *hpca, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
    hpca->hi2c = hi2c;
    hpca->addr = addr;
    PCA9685_Reset(hpca);
}

// 复位
void PCA9685_Reset(PCA9685_HandleTypeDef *hpca)
{
    PCA9685_Write8(hpca, PCA9685_MODE1, 0x00);
}

// 设置PWM频率
void PCA9685_SetPWMFreq(PCA9685_HandleTypeDef *hpca, float freq)
{
    uint8_t prescale, oldmode, newmode;
    float prescaleval;

    freq *= 0.9f;  // 修正频率过冲（参见issue #11）
    prescaleval = 25000000.0f;   // 25MHz 内部时钟
    prescaleval /= 4096.0f;
    prescaleval /= freq;
    prescaleval -= 1.0f;

    prescale = (uint8_t)(prescaleval + 0.5f);  // 四舍五入

    // 读取当前MODE1
    PCA9685_Read8(hpca, PCA9685_MODE1, &oldmode);

    // 进入睡眠模式（设置SLEEP位）
    newmode = (oldmode & 0x7F) | 0x10;
    PCA9685_Write8(hpca, PCA9685_MODE1, newmode);

    // 设置预分频器
    PCA9685_Write8(hpca, PCA9685_PRESCALE, prescale);

    // 恢复原模式
    PCA9685_Write8(hpca, PCA9685_MODE1, oldmode);
    HAL_Delay(5);

    // 启用自动递增（RESTART, AI）
    PCA9685_Write8(hpca, PCA9685_MODE1, oldmode | 0xA1);
}

// 设置PWM
void PCA9685_SetPWM(PCA9685_HandleTypeDef *hpca, uint8_t num, uint16_t on, uint16_t off)
{
    uint8_t reg = LED0_ON_L + 4 * num;
    uint8_t data[4] = {
        on & 0xFF,
        (on >> 8) & 0xFF,
        off & 0xFF,
        (off >> 8) & 0xFF
    };
    HAL_I2C_Mem_Write(hpca->hi2c, hpca->addr << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 4, HAL_MAX_DELAY);
}

// 简化设置
void PCA9685_SetPin(PCA9685_HandleTypeDef *hpca, uint8_t num, uint16_t val, uint8_t invert)
{
    // 限幅
    if (val > 4095) val = 4095;

    if (invert) {
        if (val == 0) {
            // 完全开启（低有效）
            PCA9685_SetPWM(hpca, num, 4096, 0);
        } else if (val == 4095) {
            // 完全关闭
            PCA9685_SetPWM(hpca, num, 0, 4096);
        } else {
            PCA9685_SetPWM(hpca, num, 0, 4095 - val);
        }
    } else {
        if (val == 4095) {
            // 完全开启
            PCA9685_SetPWM(hpca, num, 4096, 0);
        } else if (val == 0) {
            // 完全关闭
            PCA9685_SetPWM(hpca, num, 0, 4096);
        } else {
            PCA9685_SetPWM(hpca, num, 0, val);
        }
    }
}