# ROS2 滤波演示工程（filter-demo）

## 工程简介
本工程基于 ROS2 实现**带噪声信号生成**与**滤波处理**功能，支持两种经典滤波算法：
- 中值滤波：用于消除尖峰噪声（如传感器突发异常值）
- 一阶低通滤波：用于平滑高频噪声，保留信号趋势

工程通过 ROS2 话题通信实现模块化设计，可通过 Foxglove Studio 可视化对比原始信号与滤波结果。

## 快速开始

### 1. 克隆仓库
```zsh
# 克隆远程仓库（替换为你的 GitHub 仓库地址）
git clone https://github.com/你的用户名/ros2-develop.git

# 进入工程根目录
cd ros2-develop

# 切换到功能分支
git checkout filter
```

### 2. 编译工程
```zsh
# 编译 filter_demo 功能包
colcon build 

# 加载 ROS2 环境变量（根据终端类型选择）
# zsh 终端
source install/setup.zsh
```

### 3. 启动节点
需打开 **3 个终端**

#### 终端 1：启动带噪声信号生成节点
```zsh
# 发布 1Hz 正弦波 + 高斯噪声信号（话题：/noisy_signal）
ros2 run filter_demo data_generator
```
- 日志输出：实时打印原始信号、噪声值、带噪声信号
- 话题类型：`std_msgs/msg/Float32`


#### 终端 2：启动滤波处理节点
```zsh
# 订阅噪声信号，执行滤波并发布结果（话题：/median_filtered、/lowpass_filtered）
ros2 run filter_demo filter_node
```
- 日志输出：实时打印输入信号、中值滤波结果、低通滤波结果
- 输出话题：
  - `/median_filtered`：中值滤波后信号
  - `/lowpass_filtered`：低通滤波后信号
#### 终端3：在容器内安装foxglove-bridge

```zsh
sudo apt install ros-jazzy-foxglove-bridge
ros2 launch foxglove_bridge foxglove_bridge_launch.xml  
```

### 4. 可视化对比（Foxglove Studio）
1. 打开Foxglove Studio选择socket连接本地环境。
2. 添加「Plot」面板，分别订阅以下 3 个话题：
   - `/noisy_signal`（原始带噪声信号）
   - `/median_filtered`（中值滤波结果）
   - `/lowpass_filtered`（低通滤波结果）
3. 观察波形差异：
   - 原始信号：存在明显噪声波动
   - 中值滤波：消除尖峰噪声，波形更稳定
   - 低通滤波：信号平滑，保留整体趋势


## 工程结构
```
ros2-filter-demo/          # 工程根目录
├── src/                   # 源代码目录
│   └── filter_demo/       # 功能包（核心代码）
│       ├── src/           # 节点源代码
│       │   ├── data_generator.cpp  # 带噪声信号生成节点
│       │   └── filter_node.cpp     # 滤波处理节点
│       ├── CMakeLists.txt # 编译配置（指定依赖、可执行文件）
│       └── package.xml    # 功能包信息（依赖声明、版本等）
├── .gitignore             # Git 忽略文件（排除编译产物、IDE 配置）
└── README.md              # 工程说明文档（本文档）
```


## 关键参数调整
可根据需求修改代码中的核心参数，优化滤波效果：

| 参数                | 作用                  | 修改位置                  | 推荐值范围 |
|---------------------|-----------------------|---------------------------|------------|
| `window_size_`      | 中值滤波窗口大小      | filter_node.cpp（第18行） | 3~9（奇数）|
| `alpha_`            | 低通滤波系数          | filter_node.cpp（第20行） | 0.1~0.5    |
| 噪声标准差          | 高斯噪声幅度          | data_generator.cpp（第28行）| 0.05~0.2  |
| 信号频率            | 原始正弦波频率        | data_generator.cpp（第38行）| 0.5~5 Hz   |




## 常见问题


2. **节点启动失败：`Could not find executable`**  
   原因：未编译或环境未加载。解决方案：
   ```zsh
   # 重新编译
   colcon build --packages-select filter_demo
   # 重新加载环境
   source ./install/setup.zsh
   ```

3. **Foxglove 无法订阅话题**

没有在容器内安装foxglove-bridge
