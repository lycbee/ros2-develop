#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include <random>

class DataGenerator : public rclcpp::Node {
public:
    DataGenerator() : Node("data_generator"), count_(0) {
        // 创建发布者，话题为/noisy_signal，消息类型Float32
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/noisy_signal", 10);
        
        // 100ms定时器（10Hz频率），定时生成信号
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&DataGenerator::generate_data, this));
        
        // 初始化高斯噪声生成器（均值0，标准差0.1）
        std::random_device rd;
        gen_ = std::mt19937(rd());
        dist_ = std::normal_distribution<float>(0.0, 0.1);
    }

private:
    void generate_data() {
        // 生成1Hz正弦波（纯信号）
        float time = count_ * 0.1f;  // 累计时间（每100ms递增）
        float pure_signal = std::sin(2 * M_PI * 1.0f * time);
        
        // 叠加高斯噪声
        float noise = dist_(gen_);
        float noisy_signal = pure_signal + noise;

        // 发布带噪声的信号
        auto msg = std_msgs::msg::Float32();
        msg.data = noisy_signal;
        publisher_->publish(msg);
        
        // 打印日志（可选）
        RCLCPP_INFO(this->get_logger(), "原始信号: %.2f, 噪声: %.2f, 带噪声信号: %.2f",
                    pure_signal, noise, noisy_signal);
        count_++;
    }

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int count_;
    std::mt19937 gen_;  // 随机数生成器
    std::normal_distribution<float> dist_;  // 高斯分布
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DataGenerator>());  // 运行节点
    rclcpp::shutdown();
    return 0;
}
