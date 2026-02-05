# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

STM32F10x IMU attitude estimation system using ICM42688 6-axis sensor with Mahony quaternion-based AHRS algorithm. Replaces Euler angle calculation to avoid gimbal lock.

**Target MCU:** STM32F103C8 (Cortex-M3, 64KB Flash, 20KB RAM)
**IDE:** Keil MDK-ARM (Project.uvprojx)
**Sensor:** ICM42688 via I2C
**Algorithm:** Mahony AHRS (6-axis: accelerometer + gyroscope)

---

## Build and Flash

### Using Keil MDK
```bash
# Open project in Keil MDK
e:\Project\icm42688_quaternion\Project.uvprojx

# Build: F7 or Project → Build Target
# Flash: F8 or Flash → Download
```

### VSCode Usage
VSCode is used for editing only. Compilation and flashing must be done via Keil MDK. The `.vscode/c_cpp_properties.json` is configured with absolute paths to the parent project.

---

## Project Architecture

```
icm42688_quaternion/
├── User/           # Application code (main.c)
├── Hardware/       # Hardware drivers and algorithms
│   ├── icm42688.c/h    # ICM42688 sensor driver (I2C)
│   ├── quaternion.c/h  # Mahony AHRS quaternion implementation
│   ├── IMU_Kalman.c/h  # Legacy Kalman filter (disabled in build)
│   ├── Serial.c/h      # UART communication
│   └── myiic.c/h       # I2C software implementation
├── Library/        # STM32F10x Standard Peripheral Library
├── Start/          # CMSIS and startup files
└── System/         # STM32 system initialization
```

---

## Core Algorithm: Mahony Quaternion AHRS

### Data Flow

```
ICM42688 Sensor → Axis Remapping → Mahony_Update → Quaternion → Output
                      (optional)      (q0,q1,q2,q3)    |
                                                        → Euler (optional)
```

### Key Structures

**Quaternion_t** ([quaternion.h:43-48](Hardware/quaternion.h))
```c
typedef struct {
    float q0;  // w (scalar)
    float q1;  // x
    float q2;  // y
    float q3;  // z
} Quaternion_t;
```

**MahonyAHRS_t** ([quaternion.h:62-68](Hardware/quaternion.h))
```c
typedef struct {
    Quaternion_t quat;     // Current attitude
    float integralFBx;     // Error integral X
    float integralFBy;     // Error integral Y
    float integralFBz;     // Error integral Z
    uint8_t inited;        // Init flag
} MahonyAHRS_t;
```

### Algorithm Parameters ([quaternion.h:10-11](Hardware/quaternion.h))

```c
#define MAHONY_KP  0.5f   // Proportional gain (controls accelerometer correction)
#define MAHONY_KI  0.002f // Integral gain (controls gyro bias correction)
```

**Parameter Tuning Guide:**
- Higher KP → faster Roll/Pitch correction, but may cause oscillation
- Higher KI → faster bias correction, but may cause drift
- Current values optimized for: Roll ~2°/h, Pitch ~0.2°/h, Yaw ~11°/h

### Sensor Axis Remapping

Configure via AXIS_MAP_X/Y/Z macros ([quaternion.h:36-38](Hardware/quaternion.h)):
```c
#define AXIS_MAP_X  AXIS_MAP(0,  1)   // Output X = Sensor X, positive
#define AXIS_MAP_Y  AXIS_MAP(1,  1)   // Output Y = Sensor Y, positive
#define AXIS_MAP_Z  AXIS_MAP(2,  1)   // Output Z = Sensor Z, positive
```

Format: `AXIS_MAP(src_axis, sign)` where src_axis: 0=X,1=Y,2=Z and sign: 1=positive, -1=negative

---

## Important Constraints and Characteristics

### 6-Axis AHRS Limitations

**Yaw Drift is Inherent:** The 6-axis system (accelerometer + gyroscope) cannot observe absolute heading because gravity has no horizontal component. Yaw drifts at ~10-15°/hour due to gyro bias integration.

**For Stable Yaw:** Add magnetometer (9-axis AHRS) or accept periodic manual zeroing.

### Gimbal Lock

The internal algorithm uses quaternions (no gimbal lock), but converting to Euler angles can still exhibit gimbal lock behavior near pitch = ±90°. Prefer quaternion output for raw attitude.

### Output Format

Current output: Quaternion (q0,q1,q2,q3) via UART at 20Hz:
```
Serial_Printf("%.4f,%.4f,%.4f,%.4f\r\n", quat.q0, quat.q1, quat.q2, quat.q3);
```

For Euler angles, use `Mahony_GetEuler()` in main.c.

---

## ICM42688 Configuration

**Sensor Settings** ([icm42688.h](Hardware/icm42688.h)):
- I2C Address: 0xD2
- Accelerometer: ±2g, 200Hz ODR, 4x avg
- Gyroscope: ±500dps, 200Hz ODR, 4x avg
- Data format: Raw int16_t converted to float (g for accel, deg/s for gyro)

**Initialization Sequence** (main.c:31-70):
1. Serial/I2C/SysTick init
2. ICM42688 init
3. Wait 1s for sensor stabilization
4. Gyro bias calibration (200 samples, ~1 sec)
5. Mahony AHRS init

---

## Porting to Other MCUs

### STC32G (8051-based)

See `STC32G移植指南.md` for complete porting guide. Key changes:
- Use Keil C251 compiler instead of ARMCC
- Replace `Fast_InvSqrt()` with standard `sqrtf()` (no FPU)
- Reduce sample rate to 100Hz
- All other code portable with no changes

### Porting Checklist

1. **Hardware Layer:** Implement I2C/UART for target MCU
2. **Delay:** Provide `Delay_ms()` function
3. **Math:** Ensure `math.h` available; replace `Fast_InvSqrt()` if no hardware FPU
4. **Types:** Verify `stdint.h` support
5. **Parameters:** Adjust MAHONY_KP/KI for different sample rates

---

## Disabled Code

**IMU_Kalman.c** - Legacy Kalman filter implementation. Disabled in Keil project (`<IncludeInBuild>0</IncludeInBuild>`). Use quaternion-based Mahony algorithm instead.

---

## Testing and Debugging

### Drift Testing

Procedure for stationary drift measurement:
1. Place IMU on level surface
2. Record initial quaternion values
3. Wait 30 minutes stationary
4. Record final values and convert to Euler angles

Expected drift (current KP=0.5):
- Roll: ~2°/hour
- Pitch: ~0.2°/hour
- Yaw: ~11°/hour (inherent to 6-axis)

### Output Visualization

Use VOFA+ with FireWater format:
- Parser: `float,float,float,float` (for q0,q1,q2,q3)
- Update rate: 20Hz
