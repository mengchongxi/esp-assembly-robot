# AGENTS.md — ESP32 机器人电机控制系统

## 项目概述

这是一个基于 [ESP-IDF](https://docs.espressif.com/projects/esp-idf/) 框架 (v6.0) 构建的 **ESP32 5 自由度机械臂电机控制系统**。它提供了基于 WiFi 的多关节 PD 位置控制功能，具有 Web 用户界面、MCPWM 硬件编码器捕获和 NVS 参数持久化。

### 主要功能

- **5 关节 PD 位置控制**：100Hz FreeRTOS 控制循环，每个关节独立 PD（预留 PID）
- **MCPWM 硬件编码器**：5 个 AS5048A 绝对编码器，硬件捕获零 CPU 占用，多圈追踪
- **5 路 LEDC 电机驱动**：10 通道 PWM，20kHz 8-bit
- **零点校准**：NVS 持久化，掉电不丢失
- **Web 界面**：5 关节实时角度显示、目标设定、PD 调参、急停
- **WiFi 连接**：STA 模式，DHCP

---

## 技术栈

| 组件 | 技术 |
|------|------|
| 硬件 | ESP32 (Xtensa LX6) |
| 框架 | ESP-IDF v6.0 |
| 构建系统 | CMake + Ninja |
| RTOS | FreeRTOS |
| 语言 | C（主要逻辑）、C++（Web UI 页面） |
| 网络 | WiFi（STA 模式）、HTTP 服务器 |
| 电机驱动 | LEDC PWM |
| 编码器 | MCPWM Capture |

---

## 项目结构

```
motor_wifi_control/
├── CMakeLists.txt          # 根 CMake 配置
├── sdkconfig               # ESP-IDF 项目配置
├── robot.xml               # MuJoCo 机器人描述文件
├── dianlutu.md             # 电路引脚映射表
├── sch.md                  # 原理图引脚笔记
├── main/                   # 应用程序源代码
│   ├── CMakeLists.txt      # 组件 CMake 配置
│   ├── main.c              # 入口点，初始化各模块
│   ├── config.h            # 引脚定义、关节配置、PD 默认参数
│   ├── encoder_hw.c/h      # MCPWM Capture 硬件编码器（多圈、零点、NVS）
│   ├── motor_multi.c/h     # 5 路 LEDC PWM 电机驱动
│   ├── pd_controller.c/h   # PD/PID 控制器（摩擦补偿、积分预留）
│   ├── joint_manager.c/h   # 关节管理器（整合编码器+电机+PD+限位+NVS）
│   ├── control_task.c/h    # 100Hz FreeRTOS 控制任务
│   ├── http_server.c/h     # HTTP REST API
│   ├── web_ui.cpp/h        # 多关节控制 HTML 页面
│   └── wifi_manager.c/h    # WiFi 连接管理
└── build/                  # 构建产物
```

---

## 硬件引脚定义

在 `main/config.h` 中定义（`JOINT_PINS[]` 数组）：

| 关节 | 连接器 | IN1 (GPIO) | IN2 (GPIO) | 编码器 (GPIO) | MCPWM Group-CAP |
|------|--------|-----------|-----------|--------------|----------------|
| 1 | CN2 | 4 | 13 | 36 | G0-CAP0 |
| 2 | CN5 | 18 | 19 | 39 | G0-CAP1 |
| 3 | CN6 | 16 | 17 | 34 | G0-CAP2 |
| 4 | CN7 | 23 | 22 | 35 | G1-CAP0 |
| 5 | CN4 | 27 | 14 | 21 | G1-CAP1 |

---

## 编译和烧录命令

### 完整一键烧录命令

```bash
# 修复依赖（如遇 esptool 版本问题）
/home/chonxi/.espressif/python_env/idf6.0_py3.10_env/bin/pip install 'esptool~=5.3.dev1' --upgrade

# 设置 ESP-IDF 环境
export IDF_PATH=/home/chonxi/.espressif/v6.0/esp-idf
export IDF_PYTHON_ENV_PATH=/home/chonxi/.espressif/python_env/idf6.0_py3.10_env
source "$IDF_PATH/export.sh"

# 进入项目目录
cd motor_wifi_control

# 完整重新编译（清理后构建）
idf.py fullclean && idf.py build

# 仅编译（增量）
idf.py build

# 烧录到 ESP32
idf.py -p /dev/ttyACM0 flash

# 烧录并监控串口输出
idf.py -p /dev/ttyACM0 flash monitor

# 仅监控串口输出
idf.py -p /dev/ttyACM0 monitor
```

### 使用 ESP Conda 环境

```bash
conda activate esp

export IDF_PATH=/home/chonxi/.espressif/v6.0/esp-idf
export IDF_PYTHON_ENV_PATH=/home/chonxi/.espressif/python_env/idf6.0_py3.10_env
source "$IDF_PATH/export.sh"

cd motor_wifi_control && idf.py build
```

### 烧录信息

- **芯片型号**: ESP32-D0WD-V3 (revision v3.1)
- **端口**: `/dev/ttyACM0`
- **波特率**: 460800
- **固件大小**: ~830KB
- **分区布局**:
  - Bootloader @ 0x1000 (26KB)
  - 分区表 @ 0x8000 (3KB)
  - 应用程序 @ 0x10000 (830KB)

---

## REST API 端点

| 方法 | 路径 | 参数 | 功能 |
|------|------|------|------|
| GET | `/` | — | Web 控制界面 |
| GET | `/status` | — | 返回 5 关节状态 JSON |
| GET | `/joint` | `id=0-4&target=角度` | 设定目标角度 |
| GET | `/zero` | `id=0-4` | 校准零点 |
| GET | `/limits` | `id=0-4&min=度&max=度` | 设定限位 |
| GET | `/pd` | `id=0-4&kp=&kd=&ki=&dead=&min_pwm=` | 调 PD 参数 |
| GET | `/stop` | — | 急停所有关节 |

---

## 代码风格指南

### 命名规范

- **文件**：`snake_case.c`、`snake_case.h`
- **函数**：`module_verb_noun()`（例如 `motor_multi_set_speed()`）
- **变量**：静态模块变量 `s_variable_name`，常量 `ALL_CAPS`
- **宏**：`ALL_CAPS_WITH_UNDERSCORES`

### 注释语言

源文件中的注释使用 **简体中文**。

---

## 模块参考

### encoder_hw.c — MCPWM 硬件编码器

使用 MCPWM 捕获模块读取 5 个 AS5048A PWM 编码器，ISR 零 CPU 占用。

- `encoder_hw_init()` — 配置 MCPWM 捕获定时器和通道
- `encoder_hw_get_angle(id)` — 获取多圈角度（含零点偏移）
- `encoder_hw_get_raw_angle(id)` — 获取原始 0-360° 角度
- `encoder_hw_set_zero(id)` — 设定零点（存 NVS）
- `encoder_hw_load_zeros()` — 从 NVS 加载零点

### motor_multi.c — 5 路电机驱动

使用 LEDC（低速 8 通道 + 高速 2 通道）驱动 5 个直流电机。

- `motor_multi_init()` — 配置 3 个定时器 + 10 个通道
- `motor_multi_set_speed(id, speed)` — 设置速度 -255~255
- `motor_multi_stop(id)` / `motor_multi_stop_all()`

### pd_controller.c — PD/PID 控制器

含摩擦补偿和积分项预留。

- `pd_init(pd)` — 初始化默认参数
- `pd_compute(pd, target, current, dt)` — 计算输出 -255~255
- `pd_reset(pd)` — 重置内部状态

### joint_manager.c — 关节管理器

整合编码器+电机+PD，mutex 保护并发访问。

- `joint_manager_init()` — 加载 NVS 参数
- `joint_set_target(id, angle)` — 限位检查后设定目标
- `joint_set_zero(id)` / `joint_set_limits()` / `joint_set_pd()`
- `joint_emergency_stop()` — 急停
- `joint_update_all(dt)` — 控制循环核心（由 control_task 调用）

### control_task.c — 控制任务

100Hz FreeRTOS 任务，固定在 CPU 核心 1。

- `control_task_start()` / `control_task_stop()`

---

## NVS 存储项

命名空间：`"joints"`

| Key | 类型 | 说明 |
|-----|------|------|
| `j{N}_zero` | blob(float) | 零点偏移 |
| `j{N}_min/max` | blob(float) | 限位 |
| `j{N}_kp/kd/ki/dz/il` | blob(float) | PD 参数 |
| `j{N}_mp` | i32 | 最小 PWM |

---

*最后更新：2026-03-29*
*ESP-IDF 版本：v6.0*
*目标平台：ESP32*
*烧录端口：/dev/ttyACM0*
