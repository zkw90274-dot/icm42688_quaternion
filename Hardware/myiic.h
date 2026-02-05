/**
 * @file    myiic.h
 * @brief   软件 I2C 驱动（支持多总线配置）
 * @author  Your Name
 * @date    2025-02-05
 * @version 2.0.0
 *
 * @details
 * - 支持多实例、多引脚配置
 * - 向后兼容原有 API（使用默认实例）
 * - 适用于 STM32F10x 系列
 */

#ifndef __MYIIC_H
#define __MYIIC_H

#include "stm32f10x.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 常量定义 ==================== */
#define I2C_TIMEOUT_MS      100     // I2C 超时时间（ms）
#define I2C_DELAY_US        4       // I2C 时钟延时（us），约 100kHz

/* ==================== 数据结构体 ==================== */

/**
 * @brief 软件 I2C 配置结构体
 */
typedef struct {
    GPIO_TypeDef *scl_port;     // SCL GPIO 端口（GPIOA/GPIOB/GPIOC...）
    GPIO_TypeDef *sda_port;     // SDA GPIO 端口
    uint16_t     scl_pin;       // SCL 引脚号（GPIO_Pin_6）
    uint16_t     sda_pin;       // SDA 引脚号（GPIO_Pin_7）
    uint32_t     rcc_apb;       // RCC 时钟（RCC_APB2Periph_GPIOB）
    uint8_t      scl_pin_num;   // SCL 引脚编号（0-15），用于快速 IO
    uint8_t      sda_pin_num;   // SDA 引脚编号（0-15）
} soft_i2c_config_t;

/**
 * @brief 软件 I2C 句柄结构体
 */
typedef struct {
    GPIO_TypeDef *scl_port;     // SCL GPIO 端口
    GPIO_TypeDef *sda_port;     // SDA GPIO 端口
    uint16_t     scl_pin;       // SCL 引脚号
    uint16_t     sda_pin;       // SDA 引脚号
    uint8_t      scl_pin_num;   // SCL 引脚编号
    uint8_t      sda_pin_num;   // SDA 引脚编号
    uint8_t      inited;        // 初始化标志
} soft_i2c_t;

/* ==================== 预定义配置 ==================== */

/**
 * @brief 默认 I2C 配置（PA7=SCL, PA6=SDA）
 */
#define SOFT_I2C_DEFAULT_CONFIG  { \
    .scl_port = GPIOA, \
    .sda_port = GPIOA, \
    .scl_pin = GPIO_Pin_7, \
    .sda_pin = GPIO_Pin_6, \
    .rcc_apb = RCC_APB2Periph_GPIOA, \
    .scl_pin_num = 7, \
    .sda_pin_num = 6 \
}

/**
 * @brief 声明默认 I2C 实例（在 myiic.c 中定义）
 */
extern soft_i2c_t g_soft_i2c_default;

/* ==================== 新 API（支持多实例） ==================== */

/**
 * @brief   初始化软件 I2C（带配置）
 * @param   i2c    I2C 句柄指针
 * @param   config 配置结构体指针
 * @return  0=成功, -1=参数错误
 *
 * @example
 * @code
 * soft_i2c_t i2c1;
 * const soft_i2c_config_t cfg = SOFT_I2C_DEFAULT_CONFIG;
 * Soft_I2C_Init(&i2c1, &cfg);
 * @endcode
 */
int Soft_I2C_Init(soft_i2c_t *i2c, const soft_i2c_config_t *config);

/**
 * @brief   设置 SCL 电平
 * @param   i2c  I2C 句柄指针
 * @param   level 0=低电平, 1=高电平
 */
static inline void Soft_I2C_SetSCL(soft_i2c_t *i2c, uint8_t level)
{
    if (level)
        i2c->scl_port->BSRR = i2c->scl_pin;
    else
        i2c->scl_port->BRR  = i2c->scl_pin;
}

/**
 * @brief   设置 SDA 电平
 * @param   i2c   I2C 句柄指针
 * @param   level 0=低电平, 1=高电平
 */
static inline void Soft_I2C_SetSDA(soft_i2c_t *i2c, uint8_t level)
{
    if (level)
        i2c->sda_port->BSRR = i2c->sda_pin;
    else
        i2c->sda_port->BRR  = i2c->sda_pin;
}

/**
 * @brief   读取 SDA 电平
 * @param   i2c I2C 句柄指针
 * @return  0=低电平, 1=高电平
 */
static inline uint8_t Soft_I2C_GetSDA(soft_i2c_t *i2c)
{
    return (i2c->sda_port->IDR & i2c->sda_pin) ? 1 : 0;
}

/**
 * @brief   配置 SDA 为输入模式
 * @param   i2c I2C 句柄指针
 */
static inline void Soft_I2C_SDA_IN(soft_i2c_t *i2c)
{
    uint32_t pin = i2c->sda_pin;
    if (pin < GPIO_Pin_8) {
        i2c->sda_port->CRL = (i2c->sda_port->CRL & ~(0xF << (pin * 4))) | (0x08 << (pin * 4));
    } else {
        i2c->sda_port->CRH = (i2c->sda_port->CRH & ~(0xF << ((pin - 8) * 4))) | (0x08 << ((pin - 8) * 4));
    }
}

/**
 * @brief   配置 SDA 为输出模式
 * @param   i2c I2C 句柄指针
 */
static inline void Soft_I2C_SDA_OUT(soft_i2c_t *i2c)
{
    uint32_t pin = i2c->sda_pin;
    if (pin < GPIO_Pin_8) {
        i2c->sda_port->CRL = (i2c->sda_port->CRL & ~(0xF << (pin * 4))) | (0x03 << (pin * 4));
    } else {
        i2c->sda_port->CRH = (i2c->sda_port->CRH & ~(0xF << ((pin - 8) * 4))) | (0x03 << ((pin - 8) * 4));
    }
}

/**
 * @brief   I2C 起始信号
 * @param   i2c I2C 句柄指针
 */
void Soft_I2C_Start(soft_i2c_t *i2c);

/**
 * @brief   I2C 停止信号
 * @param   i2c I2C 句柄指针
 */
void Soft_I2C_Stop(soft_i2c_t *i2c);

/**
 * @brief   等待 ACK
 * @param   i2c I2C 句柄指针
 * @return  0=收到 ACK, 1=未收到 ACK
 */
uint8_t Soft_I2C_WaitAck(soft_i2c_t *i2c);

/**
 * @brief   发送 ACK
 * @param   i2c I2C 句柄指针
 */
void Soft_I2C_Ack(soft_i2c_t *i2c);

/**
 * @brief   发送 NACK
 * @param   i2c I2C 句柄指针
 */
void Soft_I2C_NAck(soft_i2c_t *i2c);

/**
 * @brief   发送一个字节
 * @param   i2c I2C 句柄指针
 * @param   byte 要发送的字节
 */
void Soft_I2C_SendByte(soft_i2c_t *i2c, uint8_t byte);

/**
 * @brief   接收一个字节
 * @param   i2c I2C 句柄指针
 * @param   ack 0=发送 NACK, 1=发送 ACK
 * @return  接收到的字节
 */
uint8_t Soft_I2C_ReadByte(soft_i2c_t *i2c, uint8_t ack);

/**
 * @brief   读取多个字节
 * @param   i2c    I2C 句柄指针
 * @param   dev    设备地址（7位地址左移1位）
 * @param   reg    寄存器地址
 * @param   len    读取长度
 * @param   data   数据缓冲区
 * @return  实际读取的字节数，0 表示失败
 */
uint8_t Soft_I2C_ReadBytes(soft_i2c_t *i2c, uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data);

/**
 * @brief   写入多个字节
 * @param   i2c    I2C 句柄指针
 * @param   dev    设备地址（7位地址左移1位）
 * @param   reg    寄存器地址
 * @param   len    写入长度
 * @param   data   数据缓冲区
 * @return  1=成功, 0=失败
 */
uint8_t Soft_I2C_WriteBytes(soft_i2c_t *i2c, uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data);

/**
 * @brief   从寄存器读取单个字节
 * @param   i2c    I2C 句柄指针
 * @param   dev    设备地址
 * @param   reg    寄存器地址
 * @param   data   数据指针
 * @return  1=成功, 0=失败
 */
uint8_t Soft_I2C_ReadRegByte(soft_i2c_t *i2c, uint8_t dev, uint8_t reg, uint8_t *data);

/**
 * @brief   向寄存器写入单个字节
 * @param   i2c    I2C 句柄指针
 * @param   dev    设备地址
 * @param   reg    寄存器地址
 * @param   data   要写入的数据
 * @return  1=成功, 0=失败
 */
uint8_t Soft_I2C_WriteRegByte(soft_i2c_t *i2c, uint8_t dev, uint8_t reg, uint8_t data);

/* ==================== 向后兼容 API（使用默认实例） ==================== */

/**
 * @brief   初始化默认 I2C（PB6/PB7）
 */
void IIC_Init(void);

void IIC_Start(void);
void IIC_Stop(void);
uint8_t IIC_Wait_Ack(void);
void IIC_Ack(void);
void IIC_NAck(void);
void IIC_Send_Byte(uint8_t txd);
uint8_t IIC_Read_Byte(uint8_t ack);
uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data);
uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data);
uint8_t IICwriteByte(uint8_t dev, uint8_t reg, uint8_t data);

/* ==================== 兼容旧版本宏定义 ==================== */
#define u8  uint8_t
#define TRUE  1
#define FALSE 0

#ifdef __cplusplus
}
#endif

#endif /* __MYIIC_H */
