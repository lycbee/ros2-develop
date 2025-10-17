# ROS2 电机 PID 控制与仿真工程（motor-pid-filter）

## 工程简介
本工程包含 **电机仿真模块**（`motor_simulator`）和 **PID 控制模块**（`motor_control`），实现电机转速的闭环控制功能：
- 电机仿真：模拟电机转速反馈（含微小噪声），发布转速、角度、扭矩数据
- PID 控制：通过比例-积分-微分（PID）算法，将电机转速稳定到目标值，支持动态调参
- 适配终端：默认支持 **zsh 终端**（环境配置指令已针对性优化）


## 工程结构

```
ros2_motor_control/          # 工程根目录（ROS2工作区）
├── src/                     # 源代码目录
│   ├── motor_simulator/     # 电机仿真模块
│   │   ├── include/motor_simulator/
│   │   │   └── motor_model.hpp  # 电机模型类定义（含状态更新、噪声添加）
│   │   ├── src/
│   │   │   └── motor_simulator_node.cpp  # 仿真节点（发布转速/角度/扭矩）
│   │   ├── CMakeLists.txt   # 仿真模块编译配置
│   │   └── package.xml      # 仿真模块依赖声明
│   │
│   └── motor_control/       # PID控制模块
│       ├── src/
│       │   └── pid_controller.cpp  # PID控制节点（订阅转速+发布控制指令）
│       ├── CMakeLists.txt   # 控制模块编译配置
│       └── package.xml      # 控制模块依赖声明（rclcpp、std_msgs）
├── install/                 # 编译产物目录（自动生成）
├── build/                   # 编译中间文件目录（自动生成）
└── log/                     # 日志目录（自动生成）
```


## 快速开始（zsh 终端）

### 1. 克隆仓库并切换分支
```bash
# 克隆远程仓库（替换为你的GitHub仓库地址）
git clone https://github.com/你的用户名/ros2_motor_control.git

# 进入工程根目录（ROS2工作区）
cd ros2_motor_control

# 切换到核心功能分支 motor_pid_filter
git checkout motor_pid_filter
```

### 2. 编译工程（zsh 环境）
```bash
# 编译两个核心模块（避免编译无关文件，加快速度）
colcon build --packages-select motor_simulator motor_control

# 加载ROS2环境变量（zsh终端专用指令，必须执行）
source install/setup.zsh

# （可选）将环境变量永久添加到zsh配置（避免每次打开终端重新source）
echo "source ~/ros2_motor_control/install/setup.zsh" >> ~/.zshrc

# 添加后重启终端，环境变量会自动加载
```

### 3. 启动节点（需 3 个 zsh 终端）

#### 终端 1：启动电机仿真节点
```bash
# 发布电机仿真数据（转速话题：/simulated_motor_velocity）
ros2 run motor_simulator motor_simulator_node

# 启动成功提示：[INFO] [xxxxxxxxx] [motor_simulator]: motor simualtor launch
```

#### 终端 2：启动 PID 控制节点
```bash
# 订阅转速反馈，计算PID输出并控制电机
ros2 run motor_control pid_controller

# 启动成功提示：
# [INFO] [xxxxxxxxx] [motor_pid_controller]: PID控制器启动成功！
# [INFO] [xxxxxxxxx] [motor_pid_controller]: 初始参数：Kp=0.80, Ki=0.15, Kd=0.05, 目标转速=200.0
```

#### 终端 3：（可选）查看话题 / 日志
```bash
# 查看所有活跃话题（确认数据正常发布）
ros2 topic list

# 监听转速反馈话题（实时查看当前转速）
ros2 topic echo /simulated_motor_velocity

# 查看PID控制节点日志（每1秒打印转速与控制输出）
ros2 node info /motor_pid_controller
```


## PID 参数调试（动态调参，无需重启节点）
所有参数支持通过 `ros2 param` 命令动态修改，实时生效，以下是常用调试指令（在新的 zsh 终端执行）：

### 1. 修改目标转速
```bash
# 将目标转速从200修改为350（支持0~1000范围）
ros2 param set /motor_pid_controller target_vel 350

# 验证修改结果
ros2 param get /motor_pid_controller target_vel
```

### 2. 调整 PID 核心参数

| 参数 | 作用         | 推荐调整范围  | 示例命令（zsh）                          |
|------|--------------|---------------|------------------------------------------|
| Kp   | 比例系数（响应速度） | 0.5~2.0       | `ros2 param set /motor_pid_controller kp 1.0`  |
| Ki   | 积分系数（消除静差） | 0.05~0.3      | `ros2 param set /motor_pid_controller ki 0.2`  |
| Kd   | 微分系数（抑制震荡） | 0.02~0.1      | `ros2 param set /motor_pid_controller kd 0.08` |



## 可视化调试（Foxglove Studio）
1. 打开 [Foxglove Studio](https://foxglove.dev/studio)，选择「ROS 2」→「Localhost」连接；
2. 添加「Plot」面板，订阅以下话题：
   - `/simulated_motor_velocity`：电机实际转速
   - 手动添加「Constant」曲线（值设为目标转速，如 350），对比实际与目标的差异；
3. 调整 PID 参数时，实时观察曲线变化，直到转速稳定在目标值 ±2 以内。

