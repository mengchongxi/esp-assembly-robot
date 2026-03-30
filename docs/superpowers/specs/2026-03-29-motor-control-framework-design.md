# ESP32 机器人电机控制框架设计文档

## 概述

基于 ESP32 (ESP-IDF v6.0) 的 5 自由度机械臂电机控制框架。使用 PD 位置控制 + MCPWM 硬件编码器捕获实现精确关节角度控制，通过 WiFi Web 界面进行交互。

## 硬件概览

- **MCU**: ESP32 (Xtensa LX6)
- **电机**: 5 路直流有刷电机，通过双 H 桥驱动芯片控制
- **编码器**: 5 个 AS5048A 磁性绝对编码器 (PWM 输出, 0-360°)
- **驱动芯片**: U2 (无关), U3, U4, U7 — 共 4 片双 H 桥
- **电源**: 24V→12V (U5), 24V→5V (U6)
- **传动方式**:
  - 关节 1, 5: 齿轮直驱
  - 关节 2, 3, 4: 同步带传动

## 关节-电机-编码器映射

| 关节 | 连接器 | 电机输出 | 驱动芯片-通道 | GPIO (IN1, IN2) | 编码器 | 编码器GPIO | 关节轴 |
|------|--------|----------|---------------|-----------------|--------|-----------|--------|
| 1 | CN2 | M5+M6 | U3-B | 4, 13 | SERVO1 | 36 | Z轴 (底座旋转) |
| 2 | CN5 | M9+M10 | U4-B | 18, 19 | SERVO2 | 39 | X轴 (肩部) |
| 3 | CN6 | M11+M12 | U4-A | 16, 17 | SERVO3 | 34 | -X轴 (肘部) |
| 4 | CN7 | M13+M14 | U7-A | 23, 22 | SERVO4 | 35 | X轴 (腕部) |
| 5 | CN4 | M7+M8 | U3-A | 27, 14 | SERVO5 | 21 | Z轴 (腕旋转) |
| 无关 | CN3 | M1+M2 | U2-A | 32, 33 | — | — | — |
| 无关 | CN1 | M3+M4 | U2-B | 25, 26 | — | — | — |

注：编码器安装在关节输出轴上，直接测量关节角度，无需减速比换算。

## LEDC PWM 通道分配

| 电机 | LEDC 模式 | 定时器 | 通道 (IN1) | 通道 (IN2) |
|------|-----------|--------|-----------|-----------|
| 关节1 | 低速 | Timer0 | CH0 (GPIO4) | CH1 (GPIO13) |
| 关节2 | 低速 | Timer0 | CH2 (GPIO18) | CH3 (GPIO19) |
| 关节3 | 低速 | Timer1 | CH4 (GPIO16) | CH5 (GPIO17) |
| 关节4 | 低速 | Timer1 | CH6 (GPIO23) | CH7 (GPIO22) |
| 关节5 | 高速 | Timer0 | CH0 (GPIO27) | CH1 (GPIO14) |

PWM 频率: 20kHz, 分辨率: 8-bit (0-255)

## MCPWM Capture 编码器分配

| 编码器 | MCPWM Group | 捕获通道 | GPIO |
|--------|-------------|---------|------|
| SERVO1 | Group 0 | CAP0 | 36 |
| SERVO2 | Group 0 | CAP1 | 39 |
| SERVO3 | Group 0 | CAP2 | 34 |
| SERVO4 | Group 1 | CAP0 | 35 |
| SERVO5 | Group 1 | CAP1 | 21 |

硬件捕获 PWM 高电平和总周期时间，零 CPU 占用，80MHz 精度。

## 软件架构

### FreeRTOS 任务划分

```
┌──────────────────────────────────────────────────┐
│  control_task (优先级 5, 100Hz)                    │
│  ┌──────────┐  ┌──────────────┐  ┌─────────────┐ │
│  │MCPWM ISR │→ │PD控制器 × 5  │→ │电机PWM × 5  │ │
│  │(硬件捕获) │  │(误差→输出)   │  │(LEDC输出)   │ │
│  └──────────┘  └──────────────┘  └─────────────┘ │
└──────────────────────────────────────────────────┘
         ↕ mutex 保护的共享状态
┌──────────────────────────────────────────────────┐
│  HTTP 任务 (优先级 3, ESP-IDF 管理)                │
│  ┌────────────┐  ┌───────────────────────┐       │
│  │ REST API   │  │ Web UI (HTML/JS)      │       │
│  │ 设置目标    │  │ 角度显示/零点/调参/IK │       │
│  └────────────┘  └───────────────────────┘       │
└──────────────────────────────────────────────────┘
```

### 模块结构

```
main/
├── config.h              # 引脚定义 + 5关节配置表
├── encoder_hw.c/h        # MCPWM Capture 硬件编码器读取
│                          (多圈追踪, 零点校准)
├── motor_multi.c/h       # 5路 LEDC PWM 电机驱动
├── pd_controller.c/h     # PD控制器 (含摩擦补偿)
├── joint_manager.c/h     # 关节管理器 (整合编码器+电机+PD+限位)
│                          NVS存储零点/限位/PD参数
├── control_task.c/h      # 100Hz FreeRTOS 控制任务
├── http_server.c/h       # REST API (多关节)
├── web_ui.cpp/h          # Web界面 (多关节控制面板)
├── wifi_manager.c/h      # WiFi (保持不变)
├── main.c                # 入口 + 任务创建
│
│  Phase 2 新增:
├── ik_solver.c/h         # 解析逆运动学
└── robot_kinematics.h    # 运动学参数常量
```

旧模块 `motor.c/h`、`as5048a.c/h`、`encoder.c/h` 将被新模块替换。

### 模块接口设计

#### encoder_hw.h

```c
#define NUM_JOINTS  5

// 初始化所有编码器的 MCPWM 捕获通道
void encoder_hw_init(void);

// 获取指定关节的当前角度（多圈，含零点偏移）
float encoder_hw_get_angle(int joint_id);  // joint_id: 0-4

// 获取原始角度（0-360°，无偏移）
float encoder_hw_get_raw_angle(int joint_id);

// 设定当前位置为零点（存入 NVS）
void encoder_hw_set_zero(int joint_id);

// 从 NVS 加载零点偏移
void encoder_hw_load_zeros(void);
```

#### motor_multi.h

```c
// 初始化所有电机的 LEDC PWM 通道
void motor_multi_init(void);

// 设置电机速度 (-255 ~ 255)
void motor_multi_set_speed(int joint_id, int speed);  // joint_id: 0-4

// 停止指定电机
void motor_multi_stop(int joint_id);

// 急停所有电机
void motor_multi_stop_all(void);
```

#### pd_controller.h

```c
typedef struct {
    float kp;              // 比例增益
    float kd;              // 微分增益
    float ki;              // 积分增益 (预留, 初始为 0, 即纯 PD)
    float dead_zone;       // 死区角度 (度)
    int   min_pwm;         // 最小启动 PWM (摩擦补偿)
    float prev_error;      // 上一次误差 (内部状态)
    float integral_error;  // 积分误差累积 (预留, ki>0 时生效)
    float integral_limit;  // 积分限幅 (防止积分饱和)
} pd_controller_t;

// 初始化 PD 控制器（设定默认参数）
void pd_init(pd_controller_t *pd);

// 计算 PD 输出
// 返回: -255 ~ 255 的 PWM 值
int pd_compute(pd_controller_t *pd, float target, float current, float dt);
```

#### joint_manager.h

```c
typedef struct {
    float target_angle;       // 目标角度
    float current_angle;      // 当前角度
    float min_limit;          // 最小限位 (度)
    float max_limit;          // 最大限位 (度)
    bool  enabled;            // 是否启用控制
    pd_controller_t pd;       // PD 控制器
} joint_state_t;

// 初始化关节管理器（加载 NVS 参数）
void joint_manager_init(void);

// 设定关节目标角度（自动限位检查）
bool joint_set_target(int joint_id, float angle_deg);

// 获取关节状态
const joint_state_t* joint_get_state(int joint_id);

// 校准零点
void joint_set_zero(int joint_id);

// 设定关节限位
void joint_set_limits(int joint_id, float min_deg, float max_deg);

// 设定 PD 参数
void joint_set_pd(int joint_id, float kp, float kd, float dead_zone, int min_pwm);

// 急停所有关节
void joint_emergency_stop(void);

// 控制循环更新（由 control_task 调用）
void joint_update_all(float dt);
```

### PD 控制逻辑

```
对每个关节:
  error = target_angle - current_angle
  derivative = (error - prev_error) / dt
  output = Kp × error + Kd × derivative

  // 摩擦补偿
  if |error| > dead_zone:
      if output > 0 且 output < min_pwm → output = min_pwm
      if output < 0 且 output > -min_pwm → output = -min_pwm
      output = clamp(output, -255, 255)
  else:
      output = 0  // 到位停止

  prev_error = error
  设置电机 PWM = output
```

默认 PD 参数（初始值，需实际调参）:
- Kp = 2.0
- Kd = 0.5
- Ki = 0.0（预留，初始不使用积分项）
- dead_zone = 1.0°
- min_pwm = 20
- integral_limit = 1000.0（积分限幅，防止积分饱和）

注：关节 2、3、4 受重力影响，未来可能需要启用 Ki 来消除稳态误差（重力下垂）。
结构体已预留 ki、integral_error、integral_limit 字段，启用时只需设置 ki > 0。

### 多圈角度追踪

AS5048A 输出 0-360° 绝对角度。通过检测跨零点实现多圈追踪：

```
delta = new_raw_angle - prev_raw_angle
if delta > 180° → turn_count--   // 反向跨零 (350° → 10°)
if delta < -180° → turn_count++  // 正向跨零 (10° → 350°)

effective_angle = turn_count × 360° + raw_angle - zero_offset
```

上电时 turn_count 初始化为 0，零点偏移从 NVS 加载。

### 零点校准流程

1. 用户将机械臂摆到已知的零位构型
2. 在 Web 界面点击对应关节的"设为零点"按钮
3. ESP32 记录当前编码器原始读数作为 zero_offset
4. 同时重置 turn_count = 0
5. zero_offset 存入 NVS，下次上电自动加载

### NVS 存储项

| Key 格式 | 类型 | 说明 |
|---------|------|------|
| `j{N}_zero` | float | 关节 N 的零点偏移 |
| `j{N}_min` | float | 关节 N 的最小限位 |
| `j{N}_max` | float | 关节 N 的最大限位 |
| `j{N}_kp` | float | 关节 N 的 Kp |
| `j{N}_kd` | float | 关节 N 的 Kd |
| `j{N}_ki` | float | 关节 N 的 Ki (预留) |
| `j{N}_dz` | float | 关节 N 的死区 |
| `j{N}_mp` | int32 | 关节 N 的最小 PWM |
| `j{N}_il` | float | 关节 N 的积分限幅 |

## REST API

| 方法 | 路径 | 参数 | 功能 |
|------|------|------|------|
| GET | `/status` | — | 返回所有关节状态 JSON |
| POST | `/joint` | `id=0-4&target=角度` | 设定目标角度 |
| POST | `/zero` | `id=0-4` | 校准零点 |
| POST | `/limits` | `id=0-4&min=度&max=度` | 设定限位 |
| POST | `/pd` | `id=0-4&kp=&kd=&ki=&dead=&min_pwm=` | 调 PD/PID 参数 |
| POST | `/stop` | — | 急停所有关节 |
| POST | `/ik` | `x=&y=&z=` | Phase 2: IK 目标 |

### `/status` 响应格式

```json
{
  "joints": [
    {
      "id": 0,
      "current": 45.2,
      "target": 45.0,
      "enabled": true,
      "limits": [-180, 180],
      "pd": {"kp": 2.0, "kd": 0.5, "ki": 0.0, "dead": 1.0, "min_pwm": 20}
    },
    ...
  ]
}
```

## Web 界面功能

- 每个关节一行：实时角度显示 | 目标角度输入框 | "GO" 按钮 | "零点" 按钮
- PD 调参面板（可折叠）：每个关节的 Kp/Kd/死区/最小PWM 滑块
- 急停按钮（大红色，固定在页面顶部）
- 状态轮询间隔：200ms
- Phase 2: XYZ 坐标输入框 + "移动" 按钮

## Phase 2: 解析逆运动学

### 运动学参数（从 robot.xml 提取）

```
关节轴定义:
  Joint1: axis = (0, 0, -1)   Z轴旋转 (底座)
  Joint2: axis = (1, 0, 0)    X轴旋转 (肩部)
  Joint3: axis = (-1, 0, 0)   -X轴旋转 (肘部)
  Joint4: axis = (1, 0, 0)    X轴旋转 (腕部)
  Joint5: axis = (0, 0, -1)   Z轴旋转 (腕, 位置控制不需要)

连杆长度:
  d1 = 0.026m    (base → joint1 高度)
  d2 = 0.040m    (joint1 → joint2)
  L1 = 0.145m    (joint2 → joint3, 大臂)
  L2 = 0.145m    (joint3 → joint4, 小臂)
  d5 = 0.040m    (joint4 → joint5)
  d6 = 0.026m    (joint5 → end)
```

### IK 求解策略（末端始终向下）

```
输入: (x, y, z) 目标位置
输出: θ1, θ2, θ3, θ4 (θ5 = 0)

1. θ1 = atan2(y, x)                    // 底座方位角

2. 投影到 θ1 平面:
   r = sqrt(x² + y²)                   // 水平距离
   z_eff = z - base_height             // 有效高度

3. 末端向下约束:
   // Joint4 使末端竖直向下
   // 腕部关节在 (r_wrist, z_wrist) = (r, z_eff + d_wrist)
   r_w = r
   z_w = z_eff + (d5 + d6)             // 补偿腕部到末端长度

4. 2-link IK (θ2, θ3):
   D = (r_w² + z_w² - L1² - L2²) / (2 × L1 × L2)
   θ3 = acos(D)                        // 肘部角度 (注意符号)
   θ2 = atan2(z_w, r_w) - atan2(L2×sin(θ3), L1+L2×cos(θ3))

5. θ4 = -(θ2 + θ3)                     // 保持末端竖直向下

6. 可达性检查:
   if |D| > 1 → 目标不可达，返回错误
```

## 开发阶段

### Phase 1: 精确关节角度控制
1. 更新 config.h — 5 关节引脚配置
2. 实现 encoder_hw — MCPWM Capture 编码器
3. 实现 motor_multi — 5 路电机驱动
4. 实现 pd_controller — PD + 摩擦补偿
5. 实现 joint_manager — 整合 + NVS
6. 实现 control_task — 100Hz 控制循环
7. 更新 http_server + web_ui — 多关节 Web 控制
8. 集成测试 — 单关节调参 → 多关节联动

### Phase 2: 逆运动学
9. 实现 ik_solver — 解析 IK
10. 更新 Web UI — XYZ 控制面板
11. 端到端测试 — IK 精度验证
