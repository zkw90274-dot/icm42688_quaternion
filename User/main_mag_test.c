/**
 * @file    main_mag_test.c
 * @brief   AK09911C 磁力计单独测试程序
 * @date    2025-02-05
 *
 * @接线
 * PA7 = SCL
 * PA6 = SDA
 * 3.3V = VDD
 * GND  = GND
 */

#include "stm32f10x.h"
#include "Delay.h"
#include "Serial.h"
#include "myiic.h"
#include "ak09911c.h"

void SysTick_Init(void)
{
    SysTick_Config(SystemCoreClock / 1000);
}

int main(void)
{
    Serial_Init(115200);
    IIC_Init();
    SysTick_Init();

    Serial_Printf("\r\n========================================\r\n");
    Serial_Printf("  AK09911C Magnetometer Test\r\n");
    Serial_Printf("========================================\r\n");

    // 尝试读取设备 ID
    Serial_Printf("\r\n[*] Step 1: Scanning I2C bus...\r\n");

    uint8_t test_addr;
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        uint8_t dummy;
        if (IICwriteByte(addr << 1, 0x00, 0x00) == 1) {
            Serial_Printf("    Found device at 0x%02X (7-bit: 0x%02X)\r\n", addr << 1, addr);
        }
    }

    // 初始化磁力计
    Serial_Printf("\r\n[*] Step 2: Initializing AK09911C...\r\n");

    ak09911c_t mag;
    uint8_t ret = AK09911C_Init(&mag);

    if (ret == 0) {
        Serial_Printf("[+] Magnetometer initialized!\r\n");
        Serial_Printf("    Device ID: 0x%04X\r\n", AK09911C_GetDeviceID(&mag));
    } else if (ret == 1) {
        Serial_Printf("[-] Error: No ACK from device (check wiring!)\r\n");
        Serial_Printf("    Expected address: 0x18 (8-bit)\r\n");
        Serial_Printf("    PA7 = SCL, PA6 = SDA\r\n");
        while (1) { Delay_ms(500); }
    } else if (ret == 2) {
        Serial_Printf("[-] Error: Wrong Device ID!\r\n");
        Serial_Printf("    Expected: 0x4805\r\n");
        while (1) { Delay_ms(500); }
    }

    // 设置为连续测量模式
    Serial_Printf("\r\n[*] Step 3: Setting continuous mode (10Hz)...\r\n");
    AK09911C_SetMode(&mag, AK09911C_MODE_CONT_10HZ);
    Serial_Printf("[+] Done!\r\n");

    // 读取磁力计数据
    Serial_Printf("\r\n[*] Step 4: Reading magnetometer data...\r\n");
    Serial_Printf("========================================\r\n");

    ak09911c_data_t data;
    uint32_t count = 0;

    while (1)
    {
        if (AK09911C_ReadData(&mag, &data) == 0)
        {
            Serial_Printf("[%04lu] X:%6.2f, Y:%6.2f, Z:%6.2f uT\r\n",
                          count++,
                          data.x, data.y, data.z);

            // 显示提示
            if (count == 1) {
                Serial_Printf("\r\n[*] Rotate the magnetometer to see values change\r\n");
                Serial_Printf("    Earth magnetic field is ~50 uT\r\n");
            }
        }
        else
        {
            Serial_Printf("[-] Read failed (data not ready)\r\n");
        }

        Delay_ms(100);  // 10Hz 输出
    }
}
