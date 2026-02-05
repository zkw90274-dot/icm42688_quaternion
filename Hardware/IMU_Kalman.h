#ifndef __IMU_KALMAN_H
#define __IMU_KALMAN_H

#include <stdint.h>

#define RAD_TO_DEG  57.295779513f
#define DEG_TO_RAD  0.017453292f

// 自适应零偏配置参数
#define STILL_DETECT_WINDOW      50     // 静止检测窗口大小
#define STILL_ACC_THRESHOLD      0.05f  // 加速度计静止阈值(g) ±0.05g
#define STILL_GYRO_THRESHOLD     1.0f   // 陀螺仪静止阈值(dps) ±1.0°/s
#define BIAS_UPDATE_ALPHA        0.02f  // 零偏更新系数(越小越慢但越稳)

typedef struct {
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

typedef struct {
    float angle;
    float bias;
    float rate;
    float P[2][2];
    float Q_angle;
    float Q_gyro;
    float R_measure;
} Kalman_t;

void IMU_Kalman_Init(void);
void IMU_Kalman_Update(float ax, float ay, float az,
                       float gx, float gy, float gz, float dt);
void IMU_Kalman_GetAttitude(Attitude_t *att);

typedef struct {
    float roll_bias;   // Roll轴陀螺仪偏差估计
    float pitch_bias;  // Pitch轴陀螺仪偏差估计
    float roll_P;      // Roll轴误差协方差(置信度)
    float pitch_P;     // Pitch轴误差协方差(置信度)
    uint8_t is_still;  // 是否处于静止状态
    float still_time;  // 静止持续时间(秒)
} Kalman_Debug_t;

void IMU_Kalman_GetDebug(Kalman_Debug_t *debug);

// 自适应零偏控制函数
void IMU_Kalman_ResetBias(void);           // 重置零偏为初始校准值
void IMU_Kalman_EnableAdaptiveBias(uint8_t enable);  // 使能/禁用自适应零偏

#endif
