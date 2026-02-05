#include "AK09911C.h"

/* 私有函数声明 */
static uint8_t AK09911C_ReadReg(AK09911C_Dev_t *dev, uint8_t reg);
static void AK09911C_WriteReg(AK09911C_Dev_t *dev, uint8_t reg, uint8_t val);
static uint8_t AK09911C_ReadRegs(AK09911C_Dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t len);
static void AK09911C_ApplyFilter(AK09911C_Dev_t *dev, float x, float y, float z);

/**
 * @brief  读取单个寄存器值
 * @param  dev: 传感器设备结构体
 * @param  reg: 寄存器地址
 * @retval 寄存器值（失败返回0xFF）
 */
static uint8_t AK09911C_ReadReg(AK09911C_Dev_t *dev, uint8_t reg) {
    uint8_t val = 0xFF;
    if (HAL_I2C_Master_Transmit(dev->hi2c, AK09911C_I2C_ADDR, &reg, 1, 100) != HAL_OK) {
        return val;
    }
    if (HAL_I2C_Master_Receive(dev->hi2c, AK09911C_I2C_ADDR, &val, 1, 100) != HAL_OK) {
        return 0xFF;
    }
    return val;
}

/**
 * @brief  写入单个寄存器值
 * @param  dev: 传感器设备结构体
 * @param  reg: 寄存器地址
 * @param  val: 要写入的值
 * @retval 无
 */
static void AK09911C_WriteReg(AK09911C_Dev_t *dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    HAL_I2C_Master_Transmit(dev->hi2c, AK09911C_I2C_ADDR, buf, 2, 100);
}

/**
 * @brief  读取多个寄存器值
 * @param  dev: 传感器设备结构体
 * @param  reg: 起始寄存器地址
 * @param  buf: 数据缓冲区
 * @param  len: 读取长度
 * @retval 0-成功，1-失败
 */
static uint8_t AK09911C_ReadRegs(AK09911C_Dev_t *dev, uint8_t reg, uint8_t *buf, uint8_t len) {
    if (HAL_I2C_Master_Transmit(dev->hi2c, AK09911C_I2C_ADDR, &reg, 1, 100) != HAL_OK) {
        return 1;
    }
    if (HAL_I2C_Master_Receive(dev->hi2c, AK09911C_I2C_ADDR, buf, len, 100) != HAL_OK) {
        return 1;
    }
    return 0;
}

/**
 * @brief  应用滤波算法
 * @param  dev: 传感器设备结构体
 * @param  x/y/z: 新的原始数据
 * @retval 无
 */
static void AK09911C_ApplyFilter(AK09911C_Dev_t *dev, float x, float y, float z) {
    if (dev->filterCount < dev->filterCoeff) {
        dev->filteredX += x;
        dev->filteredY += y;
        dev->filteredZ += z;
        dev->filterCount++;
    } else {
        // 均值滤波 + 滑动更新
        dev->filteredX = dev->filteredX / dev->filterCount;
        dev->filteredY = dev->filteredY / dev->filterCount;
        dev->filteredZ = dev->filteredZ / dev->filterCount;

        dev->filteredX = ((dev->filteredX * (dev->filterCoeff - 1)) + x) / dev->filterCoeff;
        dev->filteredY = ((dev->filteredY * (dev->filterCoeff - 1)) + y) / dev->filterCoeff;
        dev->filteredZ = ((dev->filteredZ * (dev->filterCoeff - 1)) + z) / dev->filterCoeff;
    }
}

/**
 * @brief  初始化传感器
 * @param  dev: 传感器设备结构体
 * @param  hi2c: I2C句柄
 * @retval 0-成功，1-连接失败，2-DeviceID错误
 */
uint8_t AK09911C_Init(AK09911C_Dev_t *dev, I2C_HandleTypeDef *hi2c) {
    dev->hi2c = hi2c;
    dev->magResolution = 0.15f;
    dev->filterEnabled = 0;
    dev->filterCoeff = 10;
    dev->filteredX = dev->filteredY = dev->filteredZ = 0.0f;
    dev->lastX = dev->lastY = dev->lastZ = 0.0f;
    dev->filterCount = 0;

    // 检查连接
    if (AK09911C_IsConnected(dev) != 0) {
        return 1;
    }

    // 复位传感器
    AK09911C_WriteReg(dev, AK09911C_REG_CNTL3, 0x01);
    HAL_Delay(100);

    // 验证DeviceID
    if (AK09911C_GetDeviceID(dev) != 0x05) {
        return 2;
    }

    // 设置为掉电模式
    AK09911C_SetMode(dev, AK09911C_MODE_POWERDOWN);
    return 0;
}

/**
 * @brief  检查传感器连接状态
 * @param  dev: 传感器设备结构体
 * @retval 0-已连接，1-未连接
 */
uint8_t AK09911C_IsConnected(AK09911C_Dev_t *dev) {
    return (HAL_I2C_IsDeviceReady(dev->hi2c, AK09911C_I2C_ADDR, 3, 100) == HAL_OK) ? 0 : 1;
}

/**
 * @brief  获取设备ID
 * @param  dev: 传感器设备结构体
 * @retval 设备ID（0x05为AK09911C）
 */
uint8_t AK09911C_GetDeviceID(AK09911C_Dev_t *dev) {
    return AK09911C_ReadReg(dev, AK09911C_REG_WIA2);
}

/**
 * @brief  设置传感器工作模式
 * @param  dev: 传感器设备结构体
 * @param  mode: 工作模式
 * @retval 无
 */
void AK09911C_SetMode(AK09911C_Dev_t *dev, uint8_t mode) {
    AK09911C_WriteReg(dev, AK09911C_REG_CNTL2, mode);
}

/**
 * @brief  使能/禁用滤波
 * @param  dev: 传感器设备结构体
 * @param  enable: true-使能，false-禁用
 * @retval 无
 */
void AK09911C_EnableFilter(AK09911C_Dev_t *dev, char enable) {
    dev->filterEnabled = enable;
    if (!enable) {
        dev->filteredX = dev->filteredY = dev->filteredZ = 0.0f;
        dev->filterCount = 0;
    }
}

/**
 * @brief  设置滤波系数
 * @param  dev: 传感器设备结构体
 * @param  coeff: 滤波系数（最小1）
 * @retval 无
 */
void AK09911C_SetFilterCoeff(AK09911C_Dev_t *dev, uint8_t coeff) {
    dev->filterCoeff = (coeff > 0) ? coeff : 1;
}

/**
 * @brief  读取原始磁场数据
 * @param  dev: 传感器设备结构体
 * @param  x/y/z: 原始数据缓冲区
 * @retval 0-成功，1-无数据，2-读取失败，3-数据溢出
 */
uint8_t AK09911C_ReadRawData(AK09911C_Dev_t *dev, int16_t *x, int16_t *y, int16_t *z) {
    uint8_t rawData[6];
    uint8_t st1 = AK09911C_ReadReg(dev, AK09911C_REG_ST1);

    // 检查数据就绪位
    if (!(st1 & 0x01)) {
        return 1;
    }

    // 读取6字节磁场数据
    if (AK09911C_ReadRegs(dev, AK09911C_REG_HXL, rawData, 6) != 0) {
        return 2;
    }

    // 拼接16位数据
    *x = ((int16_t)rawData[1] << 8) | rawData[0];
    *y = ((int16_t)rawData[3] << 8) | rawData[2];
    *z = ((int16_t)rawData[5] << 8) | rawData[4];

    // 检查数据溢出位
    uint8_t st2 = AK09911C_ReadReg(dev, AK09911C_REG_ST2);
    if (st2 & 0x08) {
        return 3;
    }

    return 0;
}

/**
 * @brief  读取微特斯拉单位磁场数据
 * @param  dev: 传感器设备结构体
 * @param  x/y/z: 微特斯拉数据缓冲区
 * @retval 0-成功，其他-失败（同ReadRawData）
 */
uint8_t AK09911C_ReadMagDataMicroT(AK09911C_Dev_t *dev, float *x, float *y, float *z) {
    int16_t rawX, rawY, rawZ;
    uint8_t ret = AK09911C_ReadRawData(dev, &rawX, &rawY, &rawZ);
    if (ret != 0) {
        return ret;
    }

    // 转换为微特斯拉
    *x = rawX * dev->magResolution;
    *y = rawY * dev->magResolution;
    *z = rawZ * dev->magResolution;

    // 应用滤波
    if (dev->filterEnabled) {
        AK09911C_ApplyFilter(dev, *x, *y, *z);
        *x = dev->filteredX;
        *y = dev->filteredY;
        *z = dev->filteredZ;
    }

    return 0;
}

/**
 * @brief  读取滤波后微特斯拉数据
 * @param  dev: 传感器设备结构体
 * @param  x/y/z: 滤波后数据缓冲区
 * @retval 0-成功，其他-失败
 */
uint8_t AK09911C_ReadFilteredMagData(AK09911C_Dev_t *dev, float *x, float *y, float *z) {
    int16_t rawX, rawY, rawZ;
    uint8_t ret = AK09911C_ReadRawData(dev, &rawX, &rawY, &rawZ);
    if (ret != 0) {
        return ret;
    }

    float fX = rawX * dev->magResolution;
    float fY = rawY * dev->magResolution;
    float fZ = rawZ * dev->magResolution;

    AK09911C_ApplyFilter(dev, fX, fY, fZ);
    *x = dev->filteredX;
    *y = dev->filteredY;
    *z = dev->filteredZ;

    return 0;
}

/**
 * @brief  读取增强滤波磁场数据
 * @param  dev: 传感器设备结构体
 * @param  x/y/z: 增强滤波数据缓冲区
 * @retval 0-成功，其他-失败
 */
uint8_t AK09911C_ReadEnhancedMagData(AK09911C_Dev_t *dev, float *x, float *y, float *z) {
    int16_t rawX, rawY, rawZ;
    uint8_t ret = AK09911C_ReadRawData(dev, &rawX, &rawY, &rawZ);
    if (ret != 0) {
        return ret;
    }

    float fX = rawX * dev->magResolution;
    float fY = rawY * dev->magResolution;
    float fZ = rawZ * dev->magResolution;

    // 一阶低通滤波
    const float alpha = 0.1f;
    dev->lastX = alpha * fX + (1.0f - alpha) * dev->lastX;
    dev->lastY = alpha * fY + (1.0f - alpha) * dev->lastY;
    dev->lastZ = alpha * fZ + (1.0f - alpha) * dev->lastZ;

    *x = dev->lastX;
    *y = dev->lastY;
    *z = dev->lastZ;

    return 0;
}

/**
 * @brief  读取磁场+温度数据
 * @param  dev: 传感器设备结构体
 * @param  x/y/z: 磁场数据缓冲区
 * @param  temp: 温度数据缓冲区（°C）
 * @retval 0-成功，1-无数据，2-读取失败，3-数据溢出
 */
uint8_t AK09911C_ReadBurstData(AK09911C_Dev_t *dev, float *x, float *y, float *z, float *temp) {
    uint8_t rawData[8];
    uint8_t st1 = AK09911C_ReadReg(dev, AK09911C_REG_ST1);

    if (!(st1 & 0x01)) {
        return 1;
    }

    // 读取8字节数据（磁场6字节+温度1字节+ST2 1字节）
    if (AK09911C_ReadRegs(dev, AK09911C_REG_HXL, rawData, 8) != 0) {
        return 2;
    }

    // 解析磁场数据
    int16_t rawX = ((int16_t)rawData[1] << 8) | rawData[0];
    int16_t rawY = ((int16_t)rawData[3] << 8) | rawData[2];
    int16_t rawZ = ((int16_t)rawData[5] << 8) | rawData[4];

    // 解析温度数据（25°C + rawTemp/0.5）
    int8_t rawTemp = (int8_t)rawData[6];
    *temp = 25.0f + (rawTemp / 0.5f);

    // 检查溢出位
    if (rawData[7] & 0x08) {
        return 3;
    }

    // 转换为微特斯拉
    *x = rawX * dev->magResolution;
    *y = rawY * dev->magResolution;
    *z = rawZ * dev->magResolution;

    return 0;
}

/**
 * @brief  计算航向角（基于X/Y轴）
 * @param  x/y: 磁场X/Y轴数据
 * @retval 航向角（0~360°）
 */
float AK09911C_CalcHeading(float x, float y) {
    float heading = atan2(y, x) * 180.0f / 3.14f;
    if (heading < 0) {
        heading += 360.0f;
    }
    return heading;
}

/**
 * @brief  计算磁场强度模值
 * @param  x/y/z: 磁场三轴数据
 * @retval 磁场强度模值（μT）
 */
float AK09911C_CalcMagnitude(float x, float y, float z) {
    return sqrt(x*x + y*y + z*z);
}

/**
 * @brief  传感器自检
 * @param  dev: 传感器设备结构体
 * @retval 0-自检通过，1-自检失败，2-读取数据失败
 */
uint8_t AK09911C_SelfTest(AK09911C_Dev_t *dev) {
    int16_t x, y, z;

    // 进入自检模式
    AK09911C_SetMode(dev, AK09911C_MODE_POWERDOWN);
    AK09911C_SetMode(dev, AK09911C_MODE_SELFTEST);
    HAL_Delay(100);

    // 读取自检数据
    if (AK09911C_ReadRawData(dev, &x, &y, &z) != 0) {
        AK09911C_SetMode(dev, AK09911C_MODE_POWERDOWN);
        return 2;
    }

    // 恢复掉电模式
    AK09911C_SetMode(dev, AK09911C_MODE_POWERDOWN);

    // 转换为微特斯拉并检查范围
    float fx = x * dev->magResolution;
    float fy = y * dev->magResolution;
    float fz = z * dev->magResolution;

    if ((fx > -50 && fx < -100) && (fy > -50 && fy < -100) && (fz > -200 && fz < -250)) {
        return 0;
    } else {
        return 1;
    }
}
