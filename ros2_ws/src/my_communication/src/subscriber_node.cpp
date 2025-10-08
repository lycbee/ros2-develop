#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 订阅者节点类，继承自ROS2的Node类
class DataSubscriber : public rclcpp::Node
{
public:
  // 构造函数，初始化节点名称
  DataSubscriber() : Node("data_subscriber")
  {
    // 创建订阅者，订阅"my_topic"主题
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "my_topic",
      10,
      std::bind(&DataSubscriber::message_callback, this, std::placeholders::_1));
      
    RCLCPP_INFO(this->get_logger(), "订阅者节点已启动，等待接收消息...");
  }

private:
  // 消息回调函数：处理接收到的消息
  void message_callback(const std_msgs::msg::String & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "收到消息: %s", msg.data.c_str());
  }
  
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;  // 订阅者对象
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                  // 初始化ROS2
  rclcpp::spin(std::make_shared<DataSubscriber>());  // 运行节点
  rclcpp::shutdown();                        // 关闭ROS2
  return 0;
}
