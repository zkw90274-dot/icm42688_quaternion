#ifndef AK09911C_H
#define AK09911C_H

#include "main.h"
#include <math.h>
#include "i2c.h"

/* 传感器I2C地址 */
#define AK09911C_I2C_ADDR        0x0C << 1  // STM32 HAL库I2C地址需左移1位（包含读写位）

/* 寄存器地址定义 */
#define AK09911C_REG_WIA1        0x00
#define AK09911C_REG_WIA2        0x01
#define AK09911C_REG_ST1         0x10
#define AK09911C_REG_HXL         0x11
#define AK09911C_REG_HXH         0x12
#define AK09911C_REG_HYL         0x13
#define AK09911C_REG_HYH         0x14
#define AK09911C_REG_HZL         0x15
#define AK09911C_REG_HZH         0x16
#define AK09911C_REG_TMPS        0x17
#define AK09911C_REG_ST2         0x18
#define AK09911C_REG_CNTL2       0x31
#define AK09911C_REG_CNTL3       0x32

/* 工作模式定义 */
#define AK09911C_MODE_POWERDOWN  0x00  // 掉电模式
#define AK09911C_MODE_SINGLE     0x01  // 单次测量模式
#define AK09911C_MODE_CONT1      0x02  // 连续测量1（10Hz）
#define AK09911C_MODE_CONT2      0x04  // 连续测量2（20Hz）
#define AK09911C_MODE_CONT3      0x06  // 连续测量3（50Hz）
#define AK09911C_MODE_SELFTEST   0x10  // 自检模式
#define AK09911C_MODE_FUSEROM    0x1F  // 读取校准参数

/* 传感器参数结构体 */
typedef struct {
    I2C_HandleTypeDef *hi2c;        // I2C句柄
    float magResolution;            // 磁场分辨率（0.15µT/LSB）
    uint8_t filterCoeff;            // 滤波系数
    char filterEnabled;             // 滤波使能标志
    float filteredX, filteredY, filteredZ;  // 滤波后数据
    float lastX, lastY, lastZ;      // 增强滤波缓存
    uint8_t filterCount;            // 滤波计数
} AK09911C_Dev_t;

/* 函数声明 */
// 初始化传感器
uint8_t AK09911C_Init(AK09911C_Dev_t *dev, I2C_HandleTypeDef *hi2c);

// 检查传感器连接状态
uint8_t AK09911C_IsConnected(AK09911C_Dev_t *dev);

// 获取设备ID
uint8_t AK09911C_GetDeviceID(AK09911C_Dev_t *dev);

// 设置传感器工作模式
void AK09911C_SetMode(AK09911C_Dev_t *dev, uint8_t mode);

// 使能/禁用滤波
void AK09911C_EnableFilter(AK09911C_Dev_t *dev, char enable);

// 设置滤波系数
void AK09911C_SetFilterCoeff(AK09911C_Dev_t *dev, uint8_t coeff);

// 读取原始磁场数据（int16_t）
uint8_t AK09911C_ReadRawData(AK09911C_Dev_t *dev, int16_t *x, int16_t *y, int16_t *z);

// 读取微特斯拉单位磁场数据
uint8_t AK09911C_ReadMagDataMicroT(AK09911C_Dev_t *dev, float *x, float *y, float *z);

// 读取滤波后微特斯拉数据
uint8_t AK09911C_ReadFilteredMagData(AK09911C_Dev_t *dev, float *x, float *y, float *z);

// 读取增强滤波数据
uint8_t AK09911C_ReadEnhancedMagData(AK09911C_Dev_t *dev, float *x, float *y, float *z);

// 读取磁场+温度数据
uint8_t AK09911C_ReadBurstData(AK09911C_Dev_t *dev, float *x, float *y, float *z, float *temp);

// 计算航向角（度）
float AK09911C_CalcHeading(float x, float y);

// 计算磁场强度模值
float AK09911C_CalcMagnitude(float x, float y, float z);

// 传感器自检
uint8_t AK09911C_SelfTest(AK09911C_Dev_t *dev);

#endif
