# ICM42688 IMU Quaternion AHRS

STM32F10x 姿态解算系统，使用 ICM42688 6轴传感器 + Mahony 四元数算法，避免万向锁问题。

## 特性

- ✅ **四元数解算** - 避免 Euler 角万向锁问题
- ✅ **Mahony AHRS 算法** - 6轴融合（加速度计 + 陀螺仪）
- ✅ **轴重映射支持** - 灵活适配传感器安装方向
- ✅ **陀螺仪零偏校准** - 启动时自动校准
- ✅ **低漂移** - Roll ~2°/h, Pitch ~0.2°/h

## 硬件

| 组件 | 型号 |
|------|------|
| MCU | STM32F103C8 (Cortex-M3) |
| IMU | ICM42688 (I2C) |
| IDE | Keil MDK-ARM |

## 引脚连接

```
ICM42688    STM32F103
--------    ----------
VCC    →    3.3V
GND    →    GND
SCL    →    PB6 (I2C1_SCL)
SDA    →    PB7 (I2C1_SDA)
```

## 编译和烧录

```bash
# 使用 Keil MDK 打开项目
Project.uvprojx

# 编译：F7
# 烧录：F8
```

## 输出格式

UART 输出四元数 (20Hz)：
```
q0,q1,q2,q3
```

示例：
```
0.7071,0.0001,-0.0002,0.7071
```

## 算法参数

`Hardware/quaternion.h`:
```c
#define MAHONY_KP  0.5f   // 比例增益
#define MAHONY_KI  0.002f // 积分增益
```

## 漂移特性

| 轴 | 漂移率 | 原因 |
|---|--------|------|
| Roll | ~2°/h | 加速度计修正 ✅ |
| Pitch | ~0.2°/h | 加速度计修正 ✅ |
| Yaw | ~11°/h | 6轴固有缺陷 ⚠️ |

> **Yaw 漂移说明：** 6轴系统无法观测绝对航向（重力在水平面无分量），如需稳定航向请添加磁力计（9轴）。

## 移植到其他MCU

- **STC32G:** 参见 `STC32G移植指南.md`
- **其他平台:** 移植 Hardware/ 目录，实现 I2C/UART/Delay

## 项目结构

```
├── User/           # 应用代码 (main.c)
├── Hardware/       # 驱动和算法
│   ├── icm42688.c/h    # ICM42688 驱动
│   ├── quaternion.c/h  # Mahony 四元数算法
│   └── Serial.c/h      # UART 通信
├── Library/        # STM32 固件库
└── System/         # 系统启动文件
```

## 许可

MIT License
