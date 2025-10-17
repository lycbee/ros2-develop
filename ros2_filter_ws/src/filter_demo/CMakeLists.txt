cmake_minimum_required(VERSION 3.8)
project(filter_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# 编译数据生成器节点
add_executable(data_generator src/data_generator.cpp)
ament_target_dependencies(data_generator rclcpp std_msgs)
target_link_libraries(data_generator m)  # 链接数学库（sin等函数需要）

# 编译滤波器节点
add_executable(filter_node src/filter_node.cpp)
ament_target_dependencies(filter_node rclcpp std_msgs)

# 安装节点到系统路径（让ros2 run能找到）
install(TARGETS
  data_generator
  filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()