#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

// 发布者节点类，继承自ROS2的Node类
class DataPublisher : public rclcpp::Node
{
public:
  // 构造函数，初始化节点名称
  DataPublisher() : Node("data_publisher"), count_(0)
  {
    // 创建发布者，主题名为"my_topic"，消息类型为String，队列长度10
    publisher_ = this->create_publisher<std_msgs::msg::String>("my_topic", 10);
    
    // 创建定时器，每2秒触发一次回调函数
    timer_ = this->create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&DataPublisher::publish_message, this));
      
    RCLCPP_INFO(this->get_logger(), "发布者节点已启动，开始发送消息...");
  }

private:
  // 定时器回调函数：生成并发布消息
  void publish_message()
  {
    auto msg = std_msgs::msg::String();
    msg.data = "当前计数: " + std::to_string(count_);
    publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "已发送: %s", msg.data.c_str());
    count_++;
  }
  
  rclcpp::TimerBase::SharedPtr timer_;       // 定时器对象
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;  // 发布者对象
  size_t count_;                             // 计数变量
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);                  // 初始化ROS2
  rclcpp::spin(std::make_shared<DataPublisher>());  // 运行节点
  rclcpp::shutdown();                        // 关闭ROS2
  return 0;
}
