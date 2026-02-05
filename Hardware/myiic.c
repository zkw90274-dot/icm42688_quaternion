/**
 * @file    myiic.c
 * @brief   软件 I2C 驱动实现（支持多总线配置）
 * @author  Your Name
 * @date    2025-02-05
 * @version 2.0.0
 */

#include "myiic.h"
#include "Delay.h"

/* ==================== 默认 I2C 实例 ==================== */
soft_i2c_t g_soft_i2c_default;

/* ==================== 新 API 实现 ==================== */

/**
 * @brief   初始化软件 I2C（带配置）
 */
int Soft_I2C_Init(soft_i2c_t *i2c, const soft_i2c_config_t *config)
{
    GPIO_InitTypeDef GPIO_InitStructure;

    if (i2c == NULL || config == NULL) {
        return -1;
    }

    // 保存配置
    i2c->scl_port     = config->scl_port;
    i2c->sda_port     = config->sda_port;
    i2c->scl_pin      = config->scl_pin;
    i2c->sda_pin      = config->sda_pin;
    i2c->scl_pin_num  = config->scl_pin_num;
    i2c->sda_pin_num  = config->sda_pin_num;
    i2c->inited       = 1;

    // 使能 GPIO 时钟
    RCC_APB2PeriphClockCmd(config->rcc_apb, ENABLE);

    // 配置 SCL 和 SDA 为开漏输出
    GPIO_InitStructure.GPIO_Pin  = config->scl_pin | config->sda_pin;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(config->scl_port, &GPIO_InitStructure);

    // 设置空闲状态（高电平）
    Soft_I2C_SetSCL(i2c, 1);
    Soft_I2C_SetSDA(i2c, 1);
    Delay_ms(10);

    return 0;
}

/**
 * @brief   I2C 起始信号
 */
void Soft_I2C_Start(soft_i2c_t *i2c)
{
    Soft_I2C_SDA_OUT(i2c);
    Soft_I2C_SetSDA(i2c, 1);
    Soft_I2C_SetSCL(i2c, 1);
    Delay_us(I2C_DELAY_US);
    Soft_I2C_SetSDA(i2c, 0);
    Delay_us(I2C_DELAY_US);
    Soft_I2C_SetSCL(i2c, 0);
}

/**
 * @brief   I2C 停止信号
 */
void Soft_I2C_Stop(soft_i2c_t *i2c)
{
    Soft_I2C_SDA_OUT(i2c);
    Soft_I2C_SetSCL(i2c, 0);
    Soft_I2C_SetSDA(i2c, 0);
    Delay_us(I2C_DELAY_US);
    Soft_I2C_SetSCL(i2c, 1);
    Soft_I2C_SetSDA(i2c, 1);
    Delay_us(I2C_DELAY_US);
}

/**
 * @brief   等待 ACK
 */
uint8_t Soft_I2C_WaitAck(soft_i2c_t *i2c)
{
    uint16_t err_time = 0;

    Soft_I2C_SDA_IN(i2c);
    Soft_I2C_SetSDA(i2c, 1);
    Delay_us(1);
    Soft_I2C_SetSCL(i2c, 1);
    Delay_us(1);

    while (Soft_I2C_GetSDA(i2c)) {
        err_time++;
        if (err_time > 250) {
            Soft_I2C_Stop(i2c);
            return 1;
        }
        Delay_us(1);
    }

    Soft_I2C_SetSCL(i2c, 0);
    return 0;
}

/**
 * @brief   发送 ACK
 */
void Soft_I2C_Ack(soft_i2c_t *i2c)
{
    Soft_I2C_SetSCL(i2c, 0);
    Soft_I2C_SDA_OUT(i2c);
    Soft_I2C_SetSDA(i2c, 0);
    Delay_us(2);
    Soft_I2C_SetSCL(i2c, 1);
    Delay_us(2);
    Soft_I2C_SetSCL(i2c, 0);
}

/**
 * @brief   发送 NACK
 */
void Soft_I2C_NAck(soft_i2c_t *i2c)
{
    Soft_I2C_SetSCL(i2c, 0);
    Soft_I2C_SDA_OUT(i2c);
    Soft_I2C_SetSDA(i2c, 1);
    Delay_us(2);
    Soft_I2C_SetSCL(i2c, 1);
    Delay_us(2);
    Soft_I2C_SetSCL(i2c, 0);
}

/**
 * @brief   发送一个字节
 */
void Soft_I2C_SendByte(soft_i2c_t *i2c, uint8_t byte)
{
    uint8_t i;

    Soft_I2C_SDA_OUT(i2c);
    Soft_I2C_SetSCL(i2c, 0);

    for (i = 0; i < 8; i++) {
        Soft_I2C_SetSDA(i2c, (byte & 0x80) >> 7);
        byte <<= 1;
        Delay_us(2);
        Soft_I2C_SetSCL(i2c, 1);
        Delay_us(2);
        Soft_I2C_SetSCL(i2c, 0);
        Delay_us(2);
    }
}

/**
 * @brief   接收一个字节
 */
uint8_t Soft_I2C_ReadByte(soft_i2c_t *i2c, uint8_t ack)
{
    uint8_t i;
    uint8_t receive = 0;

    Soft_I2C_SDA_IN(i2c);

    for (i = 0; i < 8; i++) {
        Soft_I2C_SetSCL(i2c, 0);
        Delay_us(2);
        Soft_I2C_SetSCL(i2c, 1);
        receive <<= 1;
        if (Soft_I2C_GetSDA(i2c)) {
            receive++;
        }
        Delay_us(2);
    }

    if (!ack)
        Soft_I2C_NAck(i2c);
    else
        Soft_I2C_Ack(i2c);

    return receive;
}

/**
 * @brief   读取多个字节
 */
uint8_t Soft_I2C_ReadBytes(soft_i2c_t *i2c, uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t count = 0;

    Soft_I2C_Start(i2c);
    Soft_I2C_SendByte(i2c, dev);
    if (Soft_I2C_WaitAck(i2c)) {
        Soft_I2C_Stop(i2c);
        return 0;
    }

    Soft_I2C_SendByte(i2c, reg);
    if (Soft_I2C_WaitAck(i2c)) {
        Soft_I2C_Stop(i2c);
        return 0;
    }

    Soft_I2C_Start(i2c);
    Soft_I2C_SendByte(i2c, dev + 1);  // 读取模式
    if (Soft_I2C_WaitAck(i2c)) {
        Soft_I2C_Stop(i2c);
        return 0;
    }

    for (count = 0; count < len; count++) {
        if (count != (len - 1))
            data[count] = Soft_I2C_ReadByte(i2c, 1);   // 发送 ACK
        else
            data[count] = Soft_I2C_ReadByte(i2c, 0);   // 最后一个字节发送 NACK
    }

    Soft_I2C_Stop(i2c);
    return count;
}

/**
 * @brief   写入多个字节
 */
uint8_t Soft_I2C_WriteBytes(soft_i2c_t *i2c, uint8_t dev, uint8_t reg, uint8_t len, uint8_t *data)
{
    uint8_t count = 0;

    Soft_I2C_Start(i2c);
    Soft_I2C_SendByte(i2c, dev);
    if (Soft_I2C_WaitAck(i2c)) {
        Soft_I2C_Stop(i2c);
        return 0;
    }

    Soft_I2C_SendByte(i2c, reg);
    if (Soft_I2C_WaitAck(i2c)) {
        Soft_I2C_Stop(i2c);
        return 0;
    }

    for (count = 0; count < len; count++) {
        Soft_I2C_SendByte(i2c, data[count]);
        if (Soft_I2C_WaitAck(i2c)) {
            Soft_I2C_Stop(i2c);
            return 0;
        }
    }

    Soft_I2C_Stop(i2c);
    return 1;
}

/**
 * @brief   从寄存器读取单个字节
 */
uint8_t Soft_I2C_ReadRegByte(soft_i2c_t *i2c, uint8_t dev, uint8_t reg, uint8_t *data)
{
    return Soft_I2C_ReadBytes(i2c, dev, reg, 1, data);
}

/**
 * @brief   向寄存器写入单个字节
 */
uint8_t Soft_I2C_WriteRegByte(soft_i2c_t *i2c, uint8_t dev, uint8_t reg, uint8_t data)
{
    return Soft_I2C_WriteBytes(i2c, dev, reg, 1, &data);
}

/* ==================== 向后兼容 API 实现 ==================== */

/**
 * @brief   初始化默认 I2C（PB6/PB7）
 */
void IIC_Init(void)
{
    const soft_i2c_config_t default_cfg = SOFT_I2C_DEFAULT_CONFIG;
    Soft_I2C_Init(&g_soft_i2c_default, &default_cfg);
}

void IIC_Start(void)
{
    Soft_I2C_Start(&g_soft_i2c_default);
}

void IIC_Stop(void)
{
    Soft_I2C_Stop(&g_soft_i2c_default);
}

uint8_t IIC_Wait_Ack(void)
{
    return Soft_I2C_WaitAck(&g_soft_i2c_default);
}

void IIC_Ack(void)
{
    Soft_I2C_Ack(&g_soft_i2c_default);
}

void IIC_NAck(void)
{
    Soft_I2C_NAck(&g_soft_i2c_default);
}

void IIC_Send_Byte(uint8_t txd)
{
    Soft_I2C_SendByte(&g_soft_i2c_default, txd);
}

uint8_t IIC_Read_Byte(uint8_t ack)
{
    return Soft_I2C_ReadByte(&g_soft_i2c_default, ack);
}

uint8_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    return Soft_I2C_ReadBytes(&g_soft_i2c_default, dev, reg, length, data);
}

uint8_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t length, uint8_t *data)
{
    return Soft_I2C_WriteBytes(&g_soft_i2c_default, dev, reg, length, data);
}

uint8_t IICreadByte(uint8_t dev, uint8_t reg, uint8_t *data)
{
    return Soft_I2C_ReadRegByte(&g_soft_i2c_default, dev, reg, data);
}

uint8_t IICwriteByte(uint8_t dev, uint8_t reg, uint8_t data)
{
    return Soft_I2C_WriteRegByte(&g_soft_i2c_default, dev, reg, data);
}
