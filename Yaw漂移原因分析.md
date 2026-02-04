# Yaw漂移原因深度解析

## 问题现象
- ✅ 四元数平方和 ≈ 1（归一化正常）
- ❌ 转换成欧拉角后，yaw持续漂移
- ✅ Roll 和 Pitch 相对稳定

## 根本原因：6轴AHRS无法观测Yaw

### 物理原理

加速度计只能测量**重力方向**（在静止或匀速运动时）：

```
世界坐标系：
Z轴 ↑  重力方向
    |
    |
    ----→ Y轴
   /
  ↙
 X轴

重力向量 = [0, 0, 1]  (指向地心)
```

**关键问题：**
- 重力方向只在 **Roll** 和 **Pitch** 平面上有变化
- 重力方向在 **水平面内无分量**，无法提供航向信息

### 图解说明

```
传感器绕Z轴旋转（改变Yaw）：

    Z (重力)
    ↑
    |  无论yaw=0°, 90°, 180°, 270°
    |  加速度计测量的重力都是 [0, 0, 1]
    |
    O───────→

Yaw = 0°           Yaw = 90°
  ↑                    ↑ (箭头指向自己)
  │                    │
  │                    │
  └─────→             └─────→ (传感器转向)

加速度计读数：[0, 0, 1]   加速度计读数：[0, 0, 1]  ← 完全相同！
```

---

## 为什么四元数平方和为1，Yaw还会飘？

### 四元数本身是正确的

四元数表示的是**相对旋转**，而不是绝对方向：

```
q0² + q1² + q2² + q3² = 1  ✅ 只说明旋转矩阵是正交的
                        ❌ 但不能说明航向是正确的
```

### 6轴Mahony算法的工作原理

```c
// quaternion.c 第112-115行
// 计算加速度计测量值 vs 四元数推算值的误差
ex = ay * vz - az * vy;  // ← 这些误差只能修正Roll和Pitch
ey = az * vx - ax * vz;
ez = ax * vy - ay * vx;  // ← 这个误差是重力方向的，不是航向！
```

**问题所在：**
- `ex, ey` 能修正 Roll/Pitch
- `ez` 实际上是"重力在机体坐标系Z轴的分量"，不是Yaw误差！

---

## 实际测试验证

### 测试1：静止状态
```
现象：
- Roll:  稳定在 0.02°/s 漂移  ✅
- Pitch: 稳定在 ~0°/s 漂移    ✅
- Yaw:   漂移 0.013°/s         ❌

原因：
- Roll/Pitch有加速度计修正
- Yaw只能靠陀螺仪积分，bias会导致漂移
```

### 测试2：旋转Yaw轴
```
操作：手动绕Z轴旋转传感器90°

现象：
- 四元数：q0,q1,q2,q3 都变了  ✅
- 欧拉角：Yaw = 90°  ✅ (短期内准确)

放置1分钟后：
- Yaw: 90° → 87.5°  (漂移 -2.5°)  ❌

原因：陀螺仪bias积分累积
```

---

## 解决方案

### 方案1：添加磁力计（9轴AHRS）✅ 推荐

磁力计测量地磁场方向，提供**绝对航向参考**：

```c
// 9轴Mahony算法需要磁力计数据
void Mahony_Update9Axis(MahonyAHRS_t *ahrs,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz,  // ← 磁力计
                        float dt);
```

**效果：**
- ✅ Yaw漂移 < 0.1°/分钟
- ✅ 提供绝对航向（北方为0°）

**推荐芯片：**
- IST8310（便宜，I2C）
- QMC5883L
- MMC5983MA（高精度）

---

### 方案2：定期归零（简单但不够好）

```c
// 按键时将Yaw归零
if (Button_Reset_Yaw_Pressed()) {
    ahrs->quat.q0 = 1.0f;
    ahrs->quat.q1 = 0.0f;
    ahrs->quat.q2 = 0.0f;
    ahrs->quat.q3 = 0.0f;  // 重置为单位四元数
}
```

**缺点：**
- ❌ 归零后仍然会漂移
- ❌ 无法提供绝对方向

---

### 方案3：增大KI参数（副作用大）

```c
// quaternion.h
#define MAHONY_KP 0.3f
#define MAHONY_KI 0.002f  // ← 增大到 0.01f？
```

**问题：**
- KI增大 → Roll/Pitch也会受影响
- 你之前测试过 KI=0.01 导致三轴都在飘 ❌

---

### 方案4：使用陀螺仪bias在线校准

```c
// 定期（静止时）更新陀螺仪bias
void Calibrate_GyroBias_Online(float *gx_bias, float *gy_bias, float *gz_bias)
{
    static float bias_sum[3] = {0};
    static uint16_t count = 0;

    // 检测是否静止（加速度计方差小）
    if (Is_Static()) {
        bias_sum[0] += gx - gx_bias[0];
        bias_sum[1] += gy - gy_bias[1];
        bias_sum[2] += gz - gz_bias[2];
        count++;

        if (count >= 1000) {
            gx_bias[0] += bias_sum[0] / count;
            gy_bias[1] += bias_sum[1] / count;
            gz_bias[2] += bias_sum[2] / count;
            count = 0;
            bias_sum[0] = bias_sum[1] = bias_sum[2] = 0;
        }
    }
}
```

---

## 总结

| 方案 | Yaw漂移率 | 成本 | 实现难度 | 推荐度 |
|------|----------|------|---------|--------|
| 6轴现状 | ~1°/分钟 | 无 | 简单 | ⭐⭐ |
| 9轴+磁力计 | <0.1°/分钟 | +¥5 | 中等 | ⭐⭐⭐⭐⭐ |
| 按键归零 | ~1°/分钟 | 无 | 简单 | ⭐⭐⭐ |
| Bias校准 | ~0.5°/分钟 | 无 | 复杂 | ⭐⭐⭐ |

---

## 最终建议

**如果需要稳定的航向：**
→ 添加磁力计，升级到9轴AHRS

**如果只是相对旋转：**
→ 接受现状，定期按键归零

**如果预算有限：**
→ 优化陀螺仪温度补偿，减少bias变化
