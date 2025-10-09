# ROS2 信号生成与处理项目（signal_processing）
项目名称
signal_processing - ROS2 信号生成、处理与可视化功能包
## 项目简介
本项目基于 ROS2 (Jazzy) 开发，通过 C++ 实现高频信号生成、实时信号处理与可视化验证功能。核心包含 “信号发布者”（生成正弦波、方波）和 “信号订阅者”（对信号进行逻辑处理），支持通过 Foxglove Studio 可视化信号波形，适合理解 ROS2 多话题通信、实时数据处理流程。
## 核心功能
1. 信号发布者节点（signal_publisher）

    节点名称：signal_publisher
    发布话题：
        /sine_signal：10Hz 正弦波信号（std_msgs/Float32，取值范围 [-1, 1]）
        /square_signal：1Hz 方波信号（std_msgs/Float32，取值为 1.0 或 -1.0，占空比 50%）
    发布频率：1000Hz（满足高频实时信号需求）
    实现逻辑：通过 std::sin() 生成正弦波，通过时间周期判断生成方波，定时器回调函数定时发布信号。

2. 信号订阅者节点（signal_subscriber）

    节点名称：signal_subscriber
    订阅话题：/sine_signal（正弦波）、/square_signal（方波）
    发布话题：/processed_signal（处理后信号，std_msgs/Float32）
    处理逻辑：同号保留，异号置零—— 当正弦波与方波数值同号（均为正或均为负）时，输出正弦波数值；异号时输出 0.0，实现信号门控效果。
    日志输出：接收原始信号和处理后信号时，通过 ROS2 日志打印数值，便于调试。

### 项目结构
```
plaintext

ros2_ws/                # ROS2 工作空间根目录
└── src/
    └── signal_processing/  # 功能包目录
        ├── src/
        │   ├── signal_publisher.cpp  # 信号发布者源代码
        │   └── signal_subscriber.cpp # 信号处理订阅者源代码
        ├── CMakeLists.txt          # C++ 编译配置文件（依赖、可执行文件声明）
        └── package.xml             # 功能包元信息（包名、依赖、版本等）
```

### 环境依赖

    基础环境：Ubuntu 22.04 LTS / WSL2 + ROS2 Jazzy
    核心依赖包：
        rclcpp：ROS2 C++ 核心库（节点、话题、定时器）
        std_msgs：ROS2 标准消息库（Float32 类型用于信号传输）
        cmath：C++ 数学库（生成正弦波需用到 sin() 和 M_PI）
    可视化工具：Foxglove Studio（用于实时显示信号波形）
    编译工具：colcon、cmake、g++（ROS2 容器已预装）

### 快速开始
1. 编译功能包
在 ROS2 工作空间根目录（ros2_ws）执行以下命令：
```zsh

# 编译 signal_processing 包
colcon build --packages-select signal_processing

# 加载环境变量（新终端需重新执行）
source install/setup.zsh
```
2. 启动节点（需 4 个独立终端）

终端 1：启动信号发布者
```zsh

ros2 run signal_processing signal_publisher
```
### 预期输出（1000Hz 高频日志，截取部分）：
```
plaintext

[INFO] [signal_publisher]: 发布正弦波: 0.000000, 方波: 1.000000
[INFO] [signal_publisher]: 发布正弦波: 0.062791, 方波: 1.000000
[INFO] [signal_publisher]: 发布正弦波: 0.125333, 方波: 1.000000
```
终端 2：启动信号订阅者

```zsh

# 新终端需先加载环境变量
source install/setup.zsh

# 启动订阅者
ros2 run signal_processing signal_subscriber
```
预期输出（与发布者同步）：
```
plaintext

[INFO] [signal_subscriber]: 接收正弦波: 0.000000, 方波: 1.000000 → 处理后: 0.000000
[INFO] [signal_subscriber]: 接收正弦波: 0.062791, 方波: 1.000000 → 处理后: 0.062791
[INFO] [signal_subscriber]: 接收正弦波: -0.062791, 方波: 1.000000 → 处理后: 0.000000
```
终端 3：验证话题（可选）
```zsh

source install/setup.bash

# 查看当前活跃话题
ros2 topic list
```
预期输出（包含以下 3 个信号话题）：
```
plaintext

/sine_signal
/square_signal
/processed_signal
/parameter_events
/rosout
```
终端4：在容器内安装foxglove-bridge
```zsh
sudo apt install ros-jazzy-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml  
```

3. 信号可视化（Foxglove Studio）

步骤 1：安装并启动 Foxglove Studio

    启动后点击左上角「Open Connection」
    选择websocket即可

步骤 2：添加波形图面板

  点击顶部「+」按钮 → 选择「Plot」（需添加 3 个 Plot 面板，分别对应 3 个信号）。
  配置每个面板：
      面板 1（正弦波）：点击右上角齿轮 → 「Add Topic」→ 搜索并选择 /sine_signal → 勾选 data 字段。
      面板 2（方波）：同理添加 /square_signal → 勾选 data 字段。
      面板 3（处理后信号）：添加 /processed_signal → 勾选 data 字段。

步骤 3：查看可视化效果

    正弦波面板：显示 10Hz 周期性波动曲线。
    方波面板：显示 1Hz 高低电平切换（1.0 持续 0.5 秒，-1.0 持续 0.5 秒）。
    处理后信号面板：仅在正弦波与方波同号时显示正弦波曲线，异号时显示 0 水平线，验证处理逻辑正确性。

关键技术细节

    高频信号发布：使用 rclcpp::create_wall_timer 创建 1ms 定时器（1000Hz 频率），确保信号实时性。
    信号生成逻辑：
      正弦波：std::sin(2 * M_PI * 10 * current_time)（10Hz 频率，current_time 为节点启动后的累计时间）。
      方波：(current_time - std::floor(current_time)) < 0.5 ? 1.0 : -1.0（1 秒周期，前 0.5 秒高电平，后 0.5 秒低电平）。
    多话题同步处理：订阅者通过两个独立回调函数接收正弦波和方波，用成员变量存储最新信号值，确保处理时使用 “最新一对信号”，避免数据不同步。


