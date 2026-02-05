#include "IMU_Kalman.h"
#include <math.h>

float gyro_offset_x = 0;
float gyro_offset_y = 0;
float gyro_offset_z = 0;

static Kalman_t kalman_roll;
static Kalman_t kalman_pitch;
static Attitude_t attitude;

// 自适应零偏相关变量
static float gyro_offset_initial_x = 0;  // 初始校准的零偏
static float gyro_offset_initial_y = 0;
static float gyro_offset_initial_z = 0;

static uint8_t adaptive_bias_enable = 1;  // 自适应零偏使能
static uint8_t still_detect_counter = 0;   // 静止检测计数器
static uint8_t is_still = 0;               // 当前静止状态
static float still_duration = 0.0f;        // 静止持续时间(秒)

static float Kalman_Update(Kalman_t *K, float newAngle, float newRate, float dt)
{
    K->rate = newRate - K->bias;
    K->angle += dt * K->rate;
    
    K->P[0][0] += dt * (dt * K->P[1][1] - K->P[0][1] - K->P[1][0] + K->Q_angle);
    K->P[0][1] -= dt * K->P[1][1];
    K->P[1][0] -= dt * K->P[1][1];
    K->P[1][1] += K->Q_gyro * dt;
    
    float S = K->P[0][0] + K->R_measure;
    float K_gain[2];
    K_gain[0] = K->P[0][0] / S;
    K_gain[1] = K->P[1][0] / S;
    
    float y = newAngle - K->angle;
    K->angle += K_gain[0] * y;
    K->bias  += K_gain[1] * y;
    
    float P00_temp = K->P[0][0];
    float P01_temp = K->P[0][1];
    
    K->P[0][0] -= K_gain[0] * P00_temp;
    K->P[0][1] -= K_gain[0] * P01_temp;
    K->P[1][0] -= K_gain[1] * P00_temp;
    K->P[1][1] -= K_gain[1] * P01_temp;
    
    return K->angle;
}

void IMU_Kalman_Init(void)
{
    // 无人机模式 - 飞行时加速度计不可靠，主要信任陀螺仪
    // Roll/Pitch使用相同参数，保证对称性
    kalman_roll.Q_angle = 0.0001f;     // 极低 - 最小化加速度计修正（飞行时有干扰）
    kalman_roll.Q_gyro = 1.0f;       // 极高 - 完全信任陀螺仪短期精度
    kalman_roll.R_measure = 2.0f;     // 极高 - 几乎不用加速度计修正
    kalman_roll.angle = 0.0f;
    kalman_roll.bias = 0.0f;
    kalman_roll.P[0][0] = 1.0f;
    kalman_roll.P[0][1] = 0.0f;
    kalman_roll.P[1][0] = 0.0f;
    kalman_roll.P[1][1] = 1.0f;

    kalman_pitch.Q_angle = 0.0001f;    // 极低 - 最小化加速度计修正
    kalman_pitch.Q_gyro = 0.8f;        // 适中 - 平衡响应速度和稳定性
    kalman_pitch.R_measure = 1.5f;     // 较高 - 减少噪声，提高稳定性
    kalman_pitch.angle = 0.0f;
    kalman_pitch.bias = 0.0f;
    kalman_pitch.P[0][0] = 1.0f;
    kalman_pitch.P[0][1] = 0.0f;
    kalman_pitch.P[1][0] = 0.0f;
    kalman_pitch.P[1][1] = 1.0f;

    attitude.roll = 0.0f;
    attitude.pitch = 0.0f;
    attitude.yaw = 0.0f;

    // 保存初始校准的零偏值
    gyro_offset_initial_x = gyro_offset_x;
    gyro_offset_initial_y = gyro_offset_y;
    gyro_offset_initial_z = gyro_offset_z;

    // 重置自适应零偏状态
    still_detect_counter = 0;
    is_still = 0;
    still_duration = 0.0f;
    adaptive_bias_enable = 1;
}

void IMU_Kalman_Update(float ax, float ay, float az,
                       float gx, float gy, float gz, float dt)
{
    // 陀螺仪分轴增益补偿（根据实际测试调整）
    const float gyro_gain_xy = 0.35f;  // XY轴统一增益(Roll/Pitch)
    const float gyro_gain_z = 0.6f;    // Z轴(Yaw)补偿

    gx -= gyro_offset_x;
    gy -= gyro_offset_y;
    gz -= gyro_offset_z;

    // 应用分轴增益补偿
    gx *= gyro_gain_xy;
    gy *= gyro_gain_xy;
    gz *= gyro_gain_z;

    // ==================== 静止检测 ====================
    if (adaptive_bias_enable)
    {
        // 计算加速度计模长（应该约等于1g）
        float acc_magnitude = sqrtf(ax*ax + ay*ay + az*az);
        float acc_deviation = fabsf(acc_magnitude - 1.0f);

        // 判断是否静止（加速度计稳定 + 陀螺仪接近零）
        uint8_t acc_still = (acc_deviation < STILL_ACC_THRESHOLD);
        uint8_t gyro_still = (fabsf(gx) < STILL_GYRO_THRESHOLD &&
                              fabsf(gy) < STILL_GYRO_THRESHOLD);

        if (acc_still && gyro_still)
        {
            still_detect_counter++;
            if (still_detect_counter >= STILL_DETECT_WINDOW)
            {
                still_detect_counter = STILL_DETECT_WINDOW;
                if (!is_still)
                {
                    is_still = 1;  // 进入静止状态
                }
                still_duration += dt;
            }
        }
        else
        {
            still_detect_counter = 0;
            is_still = 0;
            still_duration = 0.0f;
        }

        // ==================== 静止时自适应零偏更新 ====================
        // 当静止超过1秒时，开始缓慢更新零偏
        if (is_still && still_duration > 1.0f)
        {
            // 使用一阶低通滤波更新零偏（非常慢但稳定）
            // 新零偏 = 当前零偏 + alpha * (原始陀螺仪值 - 当前零偏)
            // 实际上就是让零偏慢慢逼近静止时的陀螺仪原始输出
            float gx_raw = gx + gyro_offset_x;
            float gy_raw = gy + gyro_offset_y;

            gyro_offset_x += BIAS_UPDATE_ALPHA * (gx_raw - gyro_offset_x);
            gyro_offset_y += BIAS_UPDATE_ALPHA * (gy_raw - gyro_offset_y);

            // 更新补偿后的陀螺仪值
            gx = gx_raw - gyro_offset_x;
            gy = gy_raw - gyro_offset_y;
        }
    }

    // 加速度计角度计算（Roll/Pitch 互不耦合）
    float acc_roll  = atan2f(ay, az) * RAD_TO_DEG;
    float acc_pitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * RAD_TO_DEG;  // 使用模长避免耦合

    attitude.roll  = Kalman_Update(&kalman_roll,  acc_roll,  gx, dt);
    attitude.pitch = Kalman_Update(&kalman_pitch, acc_pitch, gy, dt);

    attitude.yaw += gz * dt;

    // 归一化到 -180° ~ +180° 范围（避免0°/360°边界突变）
    while (attitude.yaw > 180.0f)  attitude.yaw -= 360.0f;
    while (attitude.yaw < -180.0f) attitude.yaw += 360.0f;
}

void IMU_Kalman_GetAttitude(Attitude_t *att)
{
    att->roll  = attitude.roll;
    att->pitch = attitude.pitch;
    att->yaw   = attitude.yaw;
}

void IMU_Kalman_GetDebug(Kalman_Debug_t *debug)
{
    debug->roll_bias  = kalman_roll.bias;
    debug->pitch_bias = kalman_pitch.bias;
    debug->roll_P     = kalman_roll.P[0][0];
    debug->pitch_P    = kalman_pitch.P[0][0];
    debug->is_still   = is_still;
    debug->still_time = still_duration;
}

// 重置零偏为初始校准值
void IMU_Kalman_ResetBias(void)
{
    gyro_offset_x = gyro_offset_initial_x;
    gyro_offset_y = gyro_offset_initial_y;
    gyro_offset_z = gyro_offset_initial_z;
    still_detect_counter = 0;
    is_still = 0;
    still_duration = 0.0f;
}

// 使能/禁用自适应零偏
void IMU_Kalman_EnableAdaptiveBias(uint8_t enable)
{
    adaptive_bias_enable = enable;
    if (!enable)
    {
        still_detect_counter = 0;
        is_still = 0;
        still_duration = 0.0f;
    }
}
