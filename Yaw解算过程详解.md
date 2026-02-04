# 四元数解算偏航角的完整过程

## 📍 代码位置

**文件：** [quaternion.c](../Hardware/quaternion.c)

---

## 第一部分：四元数更新过程

### 入口函数：`Mahony_Update()` (第78行)

```c
void Mahony_Update(MahonyAHRS_t *ahrs,
                   float gx, float gy, float gz,      // 陀螺仪 (rad/s)
                   float ax, float ay, float az,      // 加速度计 (已归一化)
                   float dt)                          // 时间间隔 (秒)
```

### 步骤1：计算加速度计归一化 (第96行)

```c
// 确保加速度计向量模长为1
norm = Fast_InvSqrt(ax * ax + ay * ay + az * az);
ax *= norm;
ay *= norm;
az *= norm;
```

**作用：**
- 去除加速度计的量纲，得到纯方向向量
- 例如：[0, 0, 9.8] → [0, 0, 1]

---

### 步骤2：用四元数推算重力方向 (第107-109行)

```c
// 旋转矩阵：世界坐标的重力 [0,0,1] 转换到机体坐标系
vx = 2.0f * (q1 * q3 - q0 * q2);  // 重力在机体X轴的投影
vy = 2.0f * (q0 * q1 + q2 * q3);  // 重力在机体Y轴的投影
vz = q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3;  // 重力在机体Z轴的投影
```

**数学原理：**
```
[wx]   [1-2(q2²+q3²)    2(q1q2-q0q3)    2(q1q3+q0q2)  ] [0]   [vx]
[wy] = [2(q1q2+q0q3)    1-2(q1²+q3²)    2(q2q3-q0q1)  ] [0] = [vy]
[wz]   [2(q1q3-q0q2)    2(q2q3+q0q1)    1-2(q1²+q2²)] [1]   [vz]
```

其中 [0, 0, 1] 是世界坐标系下的重力向量。

---

### 步骤3：计算方向误差 (第113-115行)

```c
// 叉乘：加速度计测量 × 推算重力 = 误差向量
ex = ay * vz - az * vy;
ey = az * vx - ax * vz;
ez = ax * vy - ay * vx;
```

**作用：**
- `ex, ey`: 重力方向误差，用于修正 Roll 和 Pitch
- `ez`: 理论上应该是 0（因为加速度计测不出 Yaw）

---

### 步骤4：PI控制器修正陀螺仪漂移 (第119-126行)

```c
// 积分项累积
ahrs->integralFBx += MAHONY_KI * ex * dt;
ahrs->integralFBy += MAHONY_KI * ey * dt;
ahrs->integralFBz += MAHONY_KI * ez * dt;

// P + I 修正
gx = gx + MAHONY_KP * ex + ahrs->integralFBx;
gy = gy + MAHONY_KP * ey + ahrs->integralFBy;
gz = gz + MAHONY_KP * ez + ahrs->integralFBz;
```

**参数：**
- `MAHONY_KP = 0.3`: 比例增益，控制修正速度
- `MAHONY_KI = 0.002`: 积分增益，修正陀螺仪零偏

---

### 步骤5：四元数微分方程更新 (第135-138行)

```c
// q_dot = 0.5 * q * ω
q0_last = q0 - 0.5f * (q1 * gx + q2 * gy + q3 * gz) * dt;
q1_last = q1 + 0.5f * (q0 * gx + q2 * gz - q3 * gy) * dt;
q2_last = q2 + 0.5f * (q0 * gy - q1 * gz + q3 * gx) * dt;
q3_last = q3 + 0.5f * (q0 * gz + q1 * gy - q2 * gx) * dt;
```

**作用：**
- 根据修正后的角速度更新四元数

---

### 步骤6：四元数归一化 (第141-146行)

```c
norm = Fast_InvSqrt(q0_last * q0_last + q1_last * q1_last +
                    q2_last * q2_last + q3_last * q3_last);
ahrs->quat.q0 = q0_last * norm;
ahrs->quat.q1 = q1_last * norm;
ahrs->quat.q2 = q2_last * norm;
ahrs->quat.q3 = q3_last * norm;
```

**结果：**
- 得到更新后的四元数 `q0, q1, q2, q3`
- 满足约束：q0² + q1² + q2² + q3² = 1

---

## 第二部分：四元数转欧拉角

### 入口函数：`Quaternion_ToEuler()` (第159行)

```c
void Quaternion_ToEuler(const Quaternion_t *quat, EulerAngle_t *euler)
{
    float q0 = quat->q0;
    float q1 = quat->q1;
    float q2 = quat->q2;
    float q3 = quat->q3;
    ...
}
```

---

### Roll 计算 (第167-168行)

```c
// Roll (φ): 绕 X 轴旋转
euler->roll = atan2f(2.0f * (q0 * q1 + q2 * q3),
                     1.0f - 2.0f * (q1 * q1 + q2 * q2));
```

**推导：**
```
tan(roll) = (2(q0q1 + q2q3)) / (1 - 2(q1² + q2²))
roll = atan2(分子, 分母)
```

---

### Pitch 计算 (第172-175行)

```c
// Pitch (θ): 绕 Y 轴旋转
float pitch_arg = 2.0f * (q0 * q2 - q3 * q1);
if (pitch_arg > 1.0f) pitch_arg = 1.0f;   // 防止 NaN
if (pitch_arg < -1.0f) pitch_arg = -1.0f;
euler->pitch = asinf(pitch_arg);
```

**推导：**
```
sin(pitch) = 2(q0q2 - q3q1)
pitch = asin(2(q0q2 - q3q1))
```

**注意：**
- 限制在 [-1, 1] 范围，防止 `asin()` 返回 NaN
- Pitch 范围：[-90°, 90°]

---

### ⭐ Yaw 计算 (第178-179行) ⭐

```c
// Yaw (ψ): 绕 Z 轴旋转
euler->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                    1.0f - 2.0f * (q2 * q2 + q3 * q3));
```

**推导：**
```
tan(yaw) = (2(q0q3 + q1q2)) / (1 - 2(q2² + q3²))
yaw = atan2(分子, 分母)
```

---

### 转角度 (第182-184行)

```c
// 弧度转角度
euler->roll *= RAD_TO_DEG;
euler->pitch *= RAD_TO_DEG;
euler->yaw *= RAD_TO_DEG;
```

**常量：**
```c
#define RAD_TO_DEG  57.295779513f  // 180 / π
```

---

## 🔄 完整数据流

```
传感器数据
    ↓
[陀螺仪 gx,gy,gz] ────┐
    │                  │
    ↓                  │
Mahony_Update()        │
    │                  │
    ├─→ 加速度计归一化
    ├─→ 推算重力方向 (用当前四元数)
    ├─→ 计算误差 (叉乘)
    ├─→ PI修正 (KP=0.3, KI=0.002)
    ├─→ 四元数微分方程 ←──────────┘
    ├─→ 归一化
    ↓
更新后的四元数 (q0, q1, q2, q3)
    ↓
Quaternion_ToEuler()
    ↓
欧拉角 (Roll, Pitch, Yaw)
```

---

## 🎯 Yaw 漂移的根本原因

### 代码层面分析

**在 `Mahony_Update()` 中：**

```c
// 第113-115行：误差计算
ex = ay * vz - az * vy;  // ← Roll 误差
ey = az * vx - ax * vz;  // ← Pitch 误差
ez = ax * vy - ay * vx;  // ← 这不是 Yaw 误差！
```

**问题所在：**
- `ez` 是重力方向在机体坐标系中偏差的 Z 分量
- 重力向量 [0, 0, 1] 绕 Z 轴旋转 **不改变**，所以 `ez` 无法提供 Yaw 修正
- **结论：** 6轴算法中，`ez` 对 Yaw 修正无效！

**在 `Quaternion_ToEuler()` 中：**

```c
// 第178-179行：Yaw 计算
euler->yaw = atan2f(2.0f * (q0 * q3 + q1 * q2),
                    1.0f - 2.0f * (q2 * q2 + q3 * q3));
```

**问题所在：**
- Yaw 完全依赖 `q0, q1, q2, q3` 的值
- 这些四元数值由陀螺仪 **积分** 得到（第135-138行）
- 陀螺仪 bias → 积累误差 → Yaw 漂移

---

## 📊 数学验证

### 四元数平方和 = 1

```c
// quaternion.c 第141行
norm = Fast_InvSqrt(q0² + q1² + q2² + q3²);
```

**含义：**
- ✅ 四元数归一化正确
- ✅ 旋转矩阵正交
- ❌ **不能保证** Yaw 绝对正确

### Yaw 只能靠陀螺仪积分

```
Yaw(t) = Yaw(0) + ∫(gz - gz_bias) dt

其中：
- gz: 陀螺仪测量值
- gz_bias: 陀螺仪零偏（随温度漂移）
- 积分时间越长，误差越大
```

---

## 💡 结论

| 问题 | 答案 |
|------|------|
| **Yaw 在哪里计算？** | `quaternion.c:178` `atan2f()` |
| **Yaw 数据来源？** | 四元数 `q0, q1, q2, q3` |
| **四元数来源？** | 陀螺仪积分 (`Mahony_Update:135-138`) |
| **为什么漂移？** | 陀螺仪 bias 累积，加速度计无法修正 |
| **四元数正确？** | 归一化正确，但相对 Yaw 仍会漂移 |

---

## 🔧 如果要解决 Yaw 漂移

**方案：添加磁力计（9轴）**

```c
// 修改 Mahony_Update，添加磁力计数据
void Mahony_Update9Axis(MahonyAHRS_t *ahrs,
                        float gx, float gy, float gz,
                        float ax, float ay, float az,
                        float mx, float my, float mz)  // ← 磁力计
{
    // 1. 磁力计提供航向参考
    // 2. 修正 Yaw 误差
    // 3. Yaw 漂移 < 0.1°/分钟
}
```
