#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <Eigen/Dense>

class PIDController : public rclcpp::Node {
public:
    PIDController() : Node("motor_pid_controller") {
        // PID参数（需自行调试，此处为示例值）
        kp_ = 0.6;   // 比例系数
        ki_ = 0.1;   // 积分系数
        kd_ = 0.03;  // 微分系数
        dt_ = 0.001; // 控制周期（1ms）
        target_velocity_ = 500.0; // 初始目标转速（可修改为200/350/500）

        // 订阅电机转速反馈
        velocity_sub_ = this->create_subscription<std_msgs::msg::Float64>(
            "simulated_motor_velocity", 10,
            std::bind(&PIDController::velocity_callback, this, std::placeholders::_1));

        // 发布控制指令到电机
        control_pub_ = this->create_publisher<std_msgs::msg::Float64>("motor_simulator_control_input", 10);

        // 定时器：周期性计算PID输出
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(dt_ * 1000)),
            std::bind(&PIDController::pid_compute, this));

        // 低通滤波参数
        alpha_ = 0.2; // 滤波系数（0~1，越小滤波越平滑）
        last_filtered_velocity_ = 0.0;
    }

private:
    // 处理电机转速反馈，并用低通滤波平滑
    void velocity_callback(const std_msgs::msg::Float64::SharedPtr msg) {
        current_velocity_ = msg->data;
        // 低通滤波公式：filtered = α·current + (1-α)·last_filtered
        filtered_velocity_ = alpha_ * current_velocity_ + (1 - alpha_) * last_filtered_velocity_;
        last_filtered_velocity_ = filtered_velocity_;
    }

    // PID计算与控制指令发布
    void pid_compute() {
        // 计算误差（目标转速 - 滤波后实际转速）
        double error = target_velocity_ - filtered_velocity_;

        // 积分项累加（消除稳态误差）
        integral_ += error * dt_;
        // 微分项（当前误差 - 上一次误差，抑制震荡）
        double derivative = (error - last_error_) / dt_;

        // PID输出 = Kp·误差 + Ki·积分 + Kd·微分
        double control_output = kp_ * error + ki_ * integral_ + kd_ * derivative;

        // 发布控制指令
        auto control_msg = std_msgs::msg::Float64();
        control_msg.data = control_output;
        control_pub_->publish(control_msg);

        // 保存当前误差，供下次微分项计算
        last_error_ = error;

        // 打印调试信息（可选）
        RCLCPP_INFO(this->get_logger(), 
                    "目标: %.1f | 原始反馈: %.1f | 滤波后: %.1f | PID输出: %.1f",
                    target_velocity_, current_velocity_, filtered_velocity_, control_output);
    }

    // PID参数
    double kp_, ki_, kd_;
    double dt_;
    double target_velocity_;
    double integral_ = 0.0;
    double last_error_ = 0.0;

    // 低通滤波参数
    double alpha_;
    double current_velocity_ = 0.0;
    double filtered_velocity_ = 0.0;
    double last_filtered_velocity_ = 0.0;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr velocity_sub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PIDController>());
    rclcpp::shutdown();
    return 0;
}
