/**
 * @file    ak09911c.h
 * @brief   AK09911C 3-axis magnetometer driver (STM32F10x)
 * @author  Your Name
 * @date    2025-02-05
 * @version 1.0.0
 *
 * @details
 * - I2C interface: Software I2C (PB6=SCL, PB7=SDA)
 * - Resolution: 0.15 µT/LSB
 * - Supports continuous measurement mode (10Hz/20Hz/50Hz)
 * - Built-in data filtering
 *
 * @note    AK09911C is often used as companion magnetometer with ICM42688
 */

#ifndef __AK09911C_H
#define __AK09911C_H

#include "stm32f10x.h"
#include <stdint.h>

/* ==================== I2C 地址定义 ==================== */
#define AK09911C_I2C_ADDR    (0x0C << 1)    // 8-bit I2C address (7-bit 0x0C << 1)

/* ==================== 寄存器地址定义 ==================== */
#define AK09911C_REG_WIA1    0x00    // Device ID low byte
#define AK09911C_REG_WIA2    0x01    // Device ID high byte (should be 0x05)
#define AK09911C_REG_ST1     0x10    // Status 1 register
#define AK09911C_REG_HXL     0x11    // X-axis data low byte
#define AK09911C_REG_HXH     0x12    // X-axis data high byte
#define AK09911C_REG_HYL     0x13    // Y-axis data low byte
#define AK09911C_REG_HYH     0x14    // Y-axis data high byte
#define AK09911C_REG_HZL     0x15    // Z-axis data low byte
#define AK09911C_REG_HZH     0x16    // Z-axis data high byte
#define AK09911C_REG_TMPS    0x17    // Temperature data
#define AK09911C_REG_ST2     0x18    // Status 2 register
#define AK09911C_REG_CNTL2   0x31    // Control 2 register (mode select)
#define AK09911C_REG_CNTL3   0x32    // Control 3 register (soft reset)

/* ==================== 工作模式定义 ==================== */
#define AK09911C_MODE_POWERDOWN  0x00    // Power down mode
#define AK09911C_MODE_SINGLE     0x01    // Single measurement mode
#define AK09911C_MODE_CONT_10HZ  0x02    // Continuous measurement (10Hz)
#define AK09911C_MODE_CONT_20HZ  0x04    // Continuous measurement (20Hz)
#define AK09911C_MODE_CONT_50HZ  0x06    // Continuous measurement (50Hz)
#define AK09911C_MODE_SELFTEST   0x10    // Self test mode

/* ==================== 数据结构体 ==================== */

/**
 * @brief   磁力计原始数据结构体
 */
typedef struct {
    int16_t x;     // X轴原始数据
    int16_t y;     // Y轴原始数据
    int16_t z;     // Z轴原始数据
} ak09911c_raw_t;

/**
 * @brief   磁力计浮点数据结构体
 */
typedef struct {
    float x;       // X轴磁场强度 (µT)
    float y;       // Y轴磁场强度 (µT)
    float z;       // Z轴磁场强度 (µT)
} ak09911c_data_t;

/**
 * @brief   磁力计传感器结构体
 */
typedef struct {
    float       resolution;     // 磁场分辨率 (0.15 µT/LSB)
    uint8_t     mode;           // 当前工作模式
    uint8_t     inited;         // 初始化标志
    float       offset_x;       // X轴硬铁校准偏移
    float       offset_y;       // Y轴硬铁校准偏移
    float       offset_z;       // Z轴硬铁校准偏移
} ak09911c_t;

/* ==================== API 函数声明 ==================== */

/**
 * @brief   初始化 AK09911C 传感器
 * @param   dev 传感器结构体指针
 * @return  0=成功, 1=连接失败, 2=设备ID错误
 */
uint8_t AK09911C_Init(ak09911c_t *dev);

/**
 * @brief   读取原始磁力计数据
 * @param   dev 传感器结构体指针
 * @param   raw 原始数据结构体指针
 * @return  0=成功, 1=数据未就绪, 2=读取失败, 3=数据溢出
 */
uint8_t AK09911C_ReadRaw(ak09911c_t *dev, ak09911c_raw_t *raw);

/**
 * @brief   读取磁力计数据（转换为 µT）
 * @param   dev 传感器结构体指针
 * @param   data 数据结构体指针
 * @return  0=成功, 其他值同 ReadRaw
 */
uint8_t AK09911C_ReadData(ak09911c_t *dev, ak09911c_data_t *data);

/**
 * @brief   设置工作模式
 * @param   dev 传感器结构体指针
 * @param   mode 工作模式
 * @return  无
 */
void AK09911C_SetMode(ak09911c_t *dev, uint8_t mode);

/**
 * @brief   复位传感器
 * @param   dev 传感器结构体指针
 * @return  无
 */
void AK09911C_Reset(ak09911c_t *dev);

/**
 * @brief   检查传感器连接状态
 * @param   dev 传感器结构体指针
 * @return  0=已连接, 1=未连接
 */
uint8_t AK09911C_IsConnected(ak09911c_t *dev);

/**
 * @brief   读取设备ID
 * @param   dev 传感器结构体指针
 * @return  设备ID (AK09911C应返回0x05)
 */
uint8_t AK09911C_GetDeviceID(ak09911c_t *dev);

/**
 * @brief   设置硬铁校准偏移
 * @param   dev 传感器结构体指针
 * @param   x, y, z 三轴偏移值
 * @return  无
 */
void AK09911C_SetOffset(ak09911c_t *dev, float x, float y, float z);

/**
 * @brief   计算航向角（绕Z轴旋转）
 * @param   mx, my 磁场X/Y分量 (µT)
 * @return  航向角 (0~360°)
 */
float AK09911C_CalcHeading(float mx, float my);

/**
 * @brief   计算磁场强度模值
 * @param   mx, my, mz 磁场三分量 (µT)
 * @return  磁场强度模值 (µT)
 */
float AK09911C_CalcMagnitude(float mx, float my, float mz);

/**
 * @brief   传感器自检
 * @param   dev 传感器结构体指针
 * @return  0=自检通过, 1=自检失败
 */
uint8_t AK09911C_SelfTest(ak09911c_t *dev);

/**
 * @brief   读取指定寄存器（调试用）
 * @param   reg 寄存器地址
 * @return  寄存器值
 */
uint8_t AK09911C_DebugReadReg(uint8_t reg);

/**
 * @brief   强制读取磁力计原始数据（跳过 DRY 检查，调试用）
 * @param   dev 传感器结构体指针
 * @param   raw 原始数据结构体指针
 * @return  0=成功
 */
uint8_t AK09911C_ForceReadRaw(ak09911c_t *dev, ak09911c_raw_t *raw);

#endif /* __AK09911C_H */
