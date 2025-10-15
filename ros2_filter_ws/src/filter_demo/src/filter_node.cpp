#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include <vector>
#include <algorithm>

class FilterNode : public rclcpp::Node {
public:
    FilterNode() : Node("filter_node"),
                   window_size_(5),  // 中值滤波窗口大小（改为size_t类型）
                   median_window_(),
                   lowpass_prev_(0.0f),
                   alpha_(0.2f) {  // 低通滤波系数（0~1，越小越平滑）
        // 订阅带噪声的信号
        subscriber_ = this->create_subscription<std_msgs::msg::Float32>(
            "/noisy_signal", 10,
            std::bind(&FilterNode::process_signal, this, std::placeholders::_1));
        
        // 创建发布者，分别发布两种滤波结果
        median_pub_ = this->create_publisher<std_msgs::msg::Float32>("/median_filtered", 10);
        lowpass_pub_ = this->create_publisher<std_msgs::msg::Float32>("/lowpass_filtered", 10);
    }

private:
    void process_signal(const std_msgs::msg::Float32::SharedPtr msg) {
        float noisy_signal = msg->data;

        // 1. 中值滤波（消除尖峰噪声）
        median_window_.push_back(noisy_signal);  // 加入新数据
        if (median_window_.size() > window_size_) {  // 类型匹配，无警告
            median_window_.erase(median_window_.begin());  // 保持窗口大小
        }
        float median_result = calculate_median();  // 计算中位数

        // 2. 一阶低通滤波（平滑信号）
        // 公式：y(n) = α*x(n) + (1-α)*y(n-1)
        float lowpass_result = alpha_ * noisy_signal + (1 - alpha_) * lowpass_prev_;
        lowpass_prev_ = lowpass_result;  // 保存当前结果用于下次计算

        // 发布滤波结果
        auto median_msg = std_msgs::msg::Float32();
        median_msg.data = median_result;
        median_pub_->publish(median_msg);

        auto lowpass_msg = std_msgs::msg::Float32();
        lowpass_msg.data = lowpass_result;
        lowpass_pub_->publish(lowpass_msg);

        // 打印日志
        RCLCPP_INFO(this->get_logger(), 
                    "输入: %.2f | 中值滤波: %.2f | 低通滤波: %.2f",
                    noisy_signal, median_result, lowpass_result);
    }

    // 计算窗口内数据的中位数
    float calculate_median() {
        if (median_window_.empty()) return 0.0f;
        std::vector<float> sorted = median_window_;
        std::sort(sorted.begin(), sorted.end());  // 排序
        size_t n = sorted.size();
        return (n % 2 == 1) ? sorted[n/2] : (sorted[n/2-1] + sorted[n/2])/2.0f;
    }

    // 成员变量（window_size_改为size_t类型）
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr subscriber_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr median_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr lowpass_pub_;
    size_t window_size_;  // 中值滤波窗口大小（修复类型不匹配警告）
    std::vector<float> median_window_;  // 滑动窗口数据
    float lowpass_prev_;  // 低通滤波上一时刻结果
    float alpha_;  // 低通滤波系数
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FilterNode>());
    rclcpp::shutdown();
    return 0;
}
