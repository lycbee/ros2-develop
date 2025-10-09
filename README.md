# ROS2 基础发布订阅通信项目（simple_communication）
项目名称
simple_communication - ROS2 基础消息收发包
项目简介
本项目基于 ROS2 (Jazzy) 开发，通过 C++ 实现了发布者（Publisher） 与订阅者（Subscriber） 两个独立节点，核心演示 ROS2 中 “发布 - 订阅” 这一异步通信模式。适合 ROS2 新手入门，帮助理解节点、话题、消息等核心概念，为复杂功能开发打下基础。

## 核心功能
1. 发布者节点（publisher）

    节点名称：data_publisher
    发布话题：/my_topic
    消息类型：std_msgs/String（ROS2 标准字符串消息）
    功能：每 2 秒生成一条带计数的字符串消息（如 “当前计数: 0”“当前计数: 1”），发布到指定话题，并通过 ROS2 日志打印发布内容。

2. 订阅者节点（subscriber）

    节点名称：data_subscriber
    订阅话题：/my_topic（与发布者话题完全一致）
    功能：实时监听目标话题，接收发布者消息后，通过 ROS2 标准日志（RCLCPP_INFO）打印接收到的内容，验证通信链路完整性。

## 项目结构
plaintext

```
ros2_ws/                # ROS2 工作空间根目录
└── src/
    └── simple_communication/  # 功能包目录
        ├── src/
        │   ├── publisher_node.cpp  # 发布者节点源代码
        │   └── subscriber_node.cpp # 订阅者节点源代码
        ├── CMakeLists.txt          # C++ 编译配置文件（声明依赖、可执行文件）
        └── package.xml             # 功能包元信息（包名、依赖、版本等）

```


### 快速开始
1. 编译功能包
在 ROS2 工作空间根目录（ros2_ws）执行以下命令：

```zsh

### 仅编译 simple_communication 包（提高编译效率）
colcon build --packages-select simple_communication

### 加载编译生成的环境变量（新终端需重新执行）
source install/setup.zsh
```

2. 启动节点（需 2 个独立终端）
终端 1：启动发布者

```zsh

ros2 run simple_communication publisher

预期输出（每 2 秒刷新一次）：
plaintext

[INFO] [data_publisher]: 已发送: 当前计数: 0
[INFO] [data_publisher]: 已发送: 当前计数: 1
[INFO] [data_publisher]: 已发送: 当前计数: 2
```

终端 2：启动订阅者

```zsh

# 新终端需先加载环境变量
source install/setup.zsh

# 启动订阅者
ros2 run simple_communication subscriber
```

### 预期输出（与发布者同步）：
```
plaintext

[INFO] [data_subscriber]: 收到消息: 当前计数: 0
[INFO] [data_subscriber]: 收到消息: 当前计数: 1
[INFO] [data_subscriber]: 收到消息: 当前计数: 2
```

### 关键概念解释

  节点（Node）：ROS2 中的最小执行单元，每个节点专注于单一功能（如 “发布消息” 或 “接收消息”），避免功能耦合。
  话题（Topic）：节点间通信的 “通道”，支持 1 对多、多对多通信（一个发布者可向多个订阅者发送消息）。
  消息（Message）：节点间传递的数据格式，ROS2 提供多种标准消息类型（如 String、Int32），也支持自定义消息。
  日志（Logger）：ROS2 提供的标准化日志系统（RCLCPP_INFO/WARN/ERROR），比 cout 更易区分节点来源，便于调试。

### 常见问题排查

  “command not found: ros2”：未进入 ROS2 容器或未加载环境变量。需通过 VS Code Dev Containers 进入容器，或执行 source /opt/ros/jazzy/setup.bash 加载全局 ROS2 环境。
  “找不到节点”：编译失败或环境变量未加载。检查 colcon build 输出是否有报错，确保 CMakeLists.txt 中正确声明了可执行文件。
  订阅者收不到消息：确认发布者与订阅者的话题名称一致（需完全匹配，区分大小写），且在同一 ROS2 环境（容器内或同一主机）。
