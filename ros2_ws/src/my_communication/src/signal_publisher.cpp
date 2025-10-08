#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include <chrono>

using namespace std::chrono_literals;

class SignalPublisher : public rclcpp::Node
{
public:
    SignalPublisher() : Node("signal_publisher")
    {
        // 创建两个发布者，分别发布正弦波和方波
        sine_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/sine_signal", 10);
        square_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/square_signal", 10);
        
        // 设置1ms定时器，实现1000Hz发布频率
        timer_ = this->create_wall_timer(
            1ms,
            std::bind(&SignalPublisher::timer_callback, this));
            
        // 记录启动时间
        start_time_ = this->now();
        RCLCPP_INFO(this->get_logger(), "信号发布者已启动，发布正弦波和方波信号");
    }

private:
    void timer_callback()
    {
        // 计算当前运行时间(秒)
        auto current_time = this->now() - start_time_;
        double t = current_time.seconds();
        
        // 生成10Hz正弦波
        std_msgs::msg::Float32 sine_msg;
        sine_msg.data = std::sin(2 * M_PI * 10 * t);
        sine_publisher_->publish(sine_msg);
        
        // 生成1Hz方波
        std_msgs::msg::Float32 square_msg;
        square_msg.data = (std::fmod(t, 1.0) < 0.5) ? 1.0 : -1.0;
        square_publisher_->publish(square_msg);
    }
    
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr sine_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr square_publisher_;
    rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalPublisher>());
    rclcpp::shutdown();
    return 0;
}
    