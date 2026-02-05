#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"
#include "myiic.h"
#include "icm42688.h"
#include "ak09911c.h"
#include "quaternion.h"

extern volatile uint32_t g_ms_tick;

void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}

static uint32_t millis(void)
{
    return g_ms_tick;
}

int main(void)
{
    Serial_Init(115200);
    IIC_Init();
    SysTick_Init();

    // ========== 9轴 AHRS 测试 ==========
    Serial_Printf("\r\n========== 9-Axis AHRS Test ==========\r\n");

    if (bsp_Icm42688Init() != 0)
    {
        while (1)
        {
            Delay_ms(500);
            Serial_Printf("ICM42688 init failed!\r\n");
        }
    }

    // 初始化磁力计
    ak09911c_t mag;
    uint8_t use_magnetometer = 0;

    if (AK09911C_Init(&mag) == 0) {
        use_magnetometer = 1;
        AK09911C_SetMode(&mag, AK09911C_MODE_SINGLE);
        Serial_Printf("Mag: Init OK\r\n");
        Delay_ms(150);

        // 验证首次读取
        ak09911c_raw_t raw_test;
        int8_t ret = AK09911C_ForceReadRaw(&mag, &raw_test);
        Serial_Printf("AHRS: Started (9-axis: %s)\r\n\r\n",
                      use_magnetometer ? "ON" : "OFF");
    } else {
        Serial_Printf("Mag: Init FAILED - 6-axis mode only\r\n");
    }

    // 陀螺仪校准（改进版：带静态检测）
    icm42688SensorData_t sensor;
    float gyro_offset[3] = {0};

    Serial_Printf("Gyro calibrating... keep sensor still!\r\n");
    Delay_ms(500);  // 等待用户放稳设备

    // 第一阶段：快速采样，检测设备是否静止
    float gyro_sum[3] = {0};
    float gyro_var[3] = {0};
    uint16_t stable_count = 0;

    for (uint16_t i = 0; i < 100; i++) {
        bsp_IcmGetAllSensorData(&sensor);
        gyro_sum[0] += sensor.gx;
        gyro_sum[1] += sensor.gy;
        gyro_sum[2] += sensor.gz;
        Delay_ms(5);
    }

    // 计算初步平均值
    gyro_offset[0] = gyro_sum[0] / 100.0f;
    gyro_offset[1] = gyro_sum[1] / 100.0f;
    gyro_offset[2] = gyro_sum[2] / 100.0f;

    // 第二阶段：精细校准，只采集静止时的数据
    for (uint16_t i = 0; i < 400; i++) {
        bsp_IcmGetAllSensorData(&sensor);

        // 检测是否静止（与平均值偏差小于 0.5 deg/s）
        if (fabsf(sensor.gx - gyro_offset[0]) < 0.5f &&
            fabsf(sensor.gy - gyro_offset[1]) < 0.5f &&
            fabsf(sensor.gz - gyro_offset[2]) < 0.5f) {
            gyro_offset[0] += sensor.gx;
            gyro_offset[1] += sensor.gy;
            gyro_offset[2] += sensor.gz;
            stable_count++;
        }
        Delay_ms(5);
    }

    if (stable_count > 0) {
        gyro_offset[0] /= stable_count;
        gyro_offset[1] /= stable_count;
        gyro_offset[2] /= stable_count;
        Serial_Printf("Gyro OK: %.2f, %.2f, %.2f dps\r\n",
                      gyro_offset[0], gyro_offset[1], gyro_offset[2]);
    } else {
        Serial_Printf("Warning: Gyro not stable during calibration!\r\n");
    }

    MahonyAHRS_t mahony;
    Mahony_Init(&mahony);

    Quaternion_t quat;
    uint32_t last_tick = millis();

    // 临时变量用于轴重映射
    float acc_src[3], acc_dst[3];
    float gyro_src[3], gyro_dst[3];
    float mag_src[3], mag_dst[3];

    // 磁力计数据
    ak09911c_data_t mag_data;

    // 主循环状态变量
    uint8_t mag_first_read_done = 0;  // 首次读取完成标志

    while (1)
    {
        uint32_t now_tick = millis();
        float dt = (now_tick - last_tick) / 1000.0f;
        last_tick = now_tick;

        // 限制 dt 范围，防止异常值
        if (dt < 0.001f) dt = 0.001f;
        if (dt > 0.1f)   dt = 0.01f;

        // 获取 IMU 传感器数据
        bsp_IcmGetAllSensorData(&sensor);

        // 应用轴重映射到加速度计
        acc_src[0] = sensor.ax;
        acc_src[1] = sensor.ay;
        acc_src[2] = sensor.az;
        Axis_Remapping(acc_src, acc_dst);

        // 陀螺仪减去零偏后再重映射
        gyro_src[0] = sensor.gx - gyro_offset[0];
        gyro_src[1] = sensor.gy - gyro_offset[1];
        gyro_src[2] = sensor.gz - gyro_offset[2];
        Axis_Remapping(gyro_src, gyro_dst);

        // 加速度计归一化
        float acc_norm = Fast_InvSqrt(acc_dst[0] * acc_dst[0] +
                                      acc_dst[1] * acc_dst[1] +
                                      acc_dst[2] * acc_dst[2]);

        // 陀螺仪转弧度
        float gx_rad = gyro_dst[0] * DEG_TO_RAD;
        float gy_rad = gyro_dst[1] * DEG_TO_RAD;
        float gz_rad = gyro_dst[2] * DEG_TO_RAD;

        // 获取磁力计数据并处理（如果可用）
        if (use_magnetometer) {
            // 静态变量
            static uint8_t mag_trigger_count = 0;
            static uint8_t mag_wait_count = 0;
            static uint8_t mag_measuring = 0;

            // 首次读取
            if (!mag_first_read_done) {
                ak09911c_raw_t raw_temp;
                int8_t ret = AK09911C_ForceReadRaw(&mag, &raw_temp);
                if (ret == 0 && (raw_temp.x != 0 || raw_temp.y != 0 || raw_temp.z != 0)) {
                    mag_data.x = raw_temp.x * 0.15f;
                    mag_data.y = raw_temp.y * 0.15f;
                    mag_data.z = raw_temp.z * 0.15f;
                    mag_first_read_done = 1;
                    mag_trigger_count = 0;
                } else {
                    mag_trigger_count = 20;
                }
            }

            // 正常测量循环：每200ms触发一次（降低I2C阻塞频率）
            if (mag_first_read_done && !mag_measuring && ++mag_trigger_count >= 40) {
                mag_trigger_count = 0;
                mag_wait_count = 0;
                mag_measuring = 1;
                AK09911C_SetMode(&mag, AK09911C_MODE_SINGLE);
            }

            // 等待测量完成
            if (mag_measuring && mag_wait_count < 4) {
                mag_wait_count++;
            }

            // 读取新数据
            if (mag_measuring && mag_wait_count >= 4) {
                ak09911c_raw_t raw_temp;
                int8_t ret = AK09911C_ForceReadRaw(&mag, &raw_temp);
                if (ret == 0 && (raw_temp.x != 0 || raw_temp.y != 0 || raw_temp.z != 0)) {
                    mag_data.x = raw_temp.x * 0.15f;
                    mag_data.y = raw_temp.y * 0.15f;
                    mag_data.z = raw_temp.z * 0.15f;
                }
                mag_measuring = 0;
            }

            // 9轴姿态解算
            mag_src[0] = mag_data.x;
            mag_src[1] = mag_data.y;
            mag_src[2] = mag_data.z;
            Axis_Remapping(mag_src, mag_dst);

            float mag_norm = Fast_InvSqrt(mag_dst[0] * mag_dst[0] +
                                          mag_dst[1] * mag_dst[1] +
                                          mag_dst[2] * mag_dst[2]);

            Mahony_Update9Axis(&mahony,
                               gx_rad, gy_rad, gz_rad,
                               acc_dst[0] * acc_norm,
                               acc_dst[1] * acc_norm,
                               acc_dst[2] * acc_norm,
                               mag_dst[0] * mag_norm,
                               mag_dst[1] * mag_norm,
                               mag_dst[2] * mag_norm,
                               dt);
        } else {
            // 6轴姿态解算
            Mahony_Update(&mahony,
                          gx_rad, gy_rad, gz_rad,
                          acc_dst[0] * acc_norm,
                          acc_dst[1] * acc_norm,
                          acc_dst[2] * acc_norm,
                          dt);
        }

        // 获取四元数姿态
        Mahony_GetQuaternion(&mahony, &quat);

        // 每20ms输出一次欧拉角（更流畅的响应）
        static uint8_t output_count = 0;
        if (++output_count >= 4) {
            output_count = 0;
            EulerAngle_t euler;
            Mahony_GetEuler(&mahony, &euler);
            Serial_Printf("%.1f,%.1f,%.1f\r\n", euler.roll, euler.pitch, euler.yaw);
        }

        Delay_ms(5);
    }
}
