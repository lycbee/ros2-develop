#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

class SignalSubscriber : public rclcpp::Node
{
public:
    SignalSubscriber() : Node("signal_subscriber")
    {
        // 订阅正弦波信号
        sine_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/sine_signal",
            10,
            std::bind(&SignalSubscriber::sine_callback, this, std::placeholders::_1));
            
        // 订阅方波信号
        square_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
            "/square_signal",
            10,
            std::bind(&SignalSubscriber::square_callback, this, std::placeholders::_1));
            
        // 创建处理后信号的发布者
        processed_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/processed_signal", 10);
        
        // 初始化信号值
        last_sine_ = 0.0f;
        last_square_ = 0.0f;
        
        RCLCPP_INFO(this->get_logger(), "信号订阅者已启动，开始处理信号");
    }

private:
    // 正弦信号回调函数
    void sine_callback(const std_msgs::msg::Float32 & msg)
    {
        last_sine_ = msg.data;
        process_and_publish();
    }
    
    // 方波信号回调函数
    void square_callback(const std_msgs::msg::Float32 & msg)
    {
        last_square_ = msg.data;
        process_and_publish();
    }
    
    // 信号处理与发布
    void process_and_publish()
    {
        std_msgs::msg::Float32 processed_msg;
        
        // 处理逻辑：当正弦波与方波同号时输出正弦波，否则输出0
        if((last_sine_ > 0 && last_square_ > 0) || (last_sine_ < 0 && last_square_ < 0))
        {
            processed_msg.data = last_sine_;
        }
        else
        {
            processed_msg.data = 0.0f;
        }
        
        processed_publisher_->publish(processed_msg);
        // 可选：输出日志
        // RCLCPP_INFO(this->get_logger(), "处理后的值: %f", processed_msg.data);
    }
    
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr sine_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr square_subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr processed_publisher_;
    float last_sine_;
    float last_square_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SignalSubscriber>());
    rclcpp::shutdown();
    return 0;
}
    