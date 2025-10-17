cmake_minimum_required(VERSION 3.8)
project(motor_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(pid_controller src/pid_controller.cpp)
ament_target_dependencies(pid_controller rclcpp std_msgs)
target_link_libraries(pid_controller Eigen3::Eigen)

install(TARGETS
  pid_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
