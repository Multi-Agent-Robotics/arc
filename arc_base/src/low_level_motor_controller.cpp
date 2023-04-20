#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "arc_msgs/msg/motor_controller_status.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <iostream>

namespace arc {

using Str = std::string;

using namespace std::placeholders;

template <typename T> using Publisher = rclcpp::Publisher<T>;
template <typename T> using Subscriber = rclcpp::Subscription<T>;

using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using VescStateStampedMsg = vesc_msgs::msg::VescStateStamped;
using float64msg = std_msgs::msg::Float64;
using MotorControllerStatusMsg = arc_msgs::msg::MotorControllerStatus;

class MotorController : public rclcpp::Node {
  private:
    double kp_;
    double ki_;
    double kd_;
    rclcpp::Time time_prev_;
    double control_ki_prev_;
    double error_eRPM_prev_;
    double vesc_state_speed_;

    float64msg msg_speed_;
    MotorControllerStatusMsg msg_status_;

    // ros
    rclcpp::TimerBase::SharedPtr timer_status_;
    rclcpp::TimerBase::SharedPtr timer_speed_;
    Subscriber<VescStateStampedMsg>::SharedPtr vesc_state_sub_;
    Subscriber<float64msg>::SharedPtr diff_drive_sub_;
    Publisher<float64msg>::SharedPtr motor_pub_;
    Publisher<MotorControllerStatusMsg>::SharedPtr status_pub_;

  public:
    MotorController(double kp, double ki, double kd)
        : Node("motor_controller"), kp_{kp}, ki_{ki}, kd_{kd},
          control_ki_prev_{0.0}, error_eRPM_prev_{0.0}, msg_speed_{},
          msg_status_{} {

        time_prev_ = this->get_clock()->now();
        this->declare_parameter("motor_id");
        auto motor_id_param = this->get_parameter("motor_id");
        std::string motor_id = motor_id_param.as_string();
        // if (motor_id != Str("left") || motor_id != Str("right")) {
        //     RCLCPP_ERROR(this->get_logger(), "motor_id != \"left\" | \"right\", motor_id: %s", motor_id.c_str());
        //     rclcpp::shutdown();
        // } 

        this->declare_parameter("controller_rate");
        auto controller_rate_param = this->get_parameter("controller_rate");
        int controller_rate = controller_rate_param.as_int();
            
        std::string motor_prefix = "motor_" + motor_id;
        // RCLCPP_INFO(this->get_logger(), "motor_prefix: %s\n", motor_prefix.c_str());

        std::string diff_drive_topic =
            motor_prefix + std::string("/target/motor/speed");
        diff_drive_sub_ = this->create_subscription<float64msg>(
            diff_drive_topic, 10, std::bind(&MotorController::step, this, std::placeholders::_1));

        std::string motor_topic = motor_prefix + "/commands/motor/speed";
        motor_pub_ = this->create_publisher<float64msg>(motor_topic, rclcpp::QoS(10).reliable());

        std::string status_topic = motor_prefix + "/controller/status";
        status_pub_ = this->create_publisher<MotorControllerStatusMsg>(status_topic, rclcpp::QoS(10).reliable());

        std::string state_topic = motor_prefix + "/sensors/core";
        vesc_state_sub_ = this->create_subscription<VescStateStampedMsg>(
            state_topic, 10,
            std::bind(&MotorController::vesc_state_cb, this, _1));

        timer_speed_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / controller_rate), [this]() {
                // RCLCPP_INFO(this->get_logger(), "msg_speed_.data: %f\n", msg_speed_.data);
                motor_pub_->publish(msg_speed_);
            });
        
        timer_status_ = this->create_wall_timer(
            std::chrono::milliseconds(1000 / controller_rate), [this]() {
                // RCLCPP_INFO(this->get_logger(), "msg_status_.control.target: %f\n", msg_status_.control.target);
                status_pub_->publish(msg_status_);
            });
    }

    void vesc_state_cb(const VescStateStampedMsg::SharedPtr msg) {
        vesc_state_speed_ = msg->state.speed;
    }

    void step(const float64msg::SharedPtr msg) {
        double target_eRPM = msg->data;
        double actual_eRPM = vesc_state_speed_;
        // std::printf("target_eRPM: %f, actual_eRPM: %f\n", target_eRPM, actual_eRPM);

        auto time_now = this->get_clock()->now();
        auto delta_time = (time_now - time_prev_).nanoseconds() * 1e-9;
        auto error_eRPM = (target_eRPM - actual_eRPM) * delta_time;
        // std::printf("error_eRPM: %f\n", error_eRPM);

        double control_kp = kp_ * error_eRPM;
        double control_ki = control_ki_prev_ + ki_ * error_eRPM;
        double control_kd = (error_eRPM_prev_ - error_eRPM) * kd_;
        // std::printf("control_kp: %f, control_ki: %f, control_kd: %f\n", control_kp, control_ki, control_kd);

        control_ki_prev_ = control_ki;
        error_eRPM_prev_ = error_eRPM;
        time_prev_ = time_now;

        msg_speed_.data = control_kp + control_ki + control_kd;

        msg_status_.control.target = target_eRPM;
        msg_status_.control.actual = actual_eRPM;
        msg_status_.control.error = error_eRPM;
        msg_status_.pid.p = control_kp;
        msg_status_.pid.i = control_ki;
        msg_status_.pid.d = control_kd;
    }
};
} // namespace arc


int main(int argc, char **argv) {
    using namespace arc;

    // printf("Starting motor controller node");
    rclcpp::init(argc, argv);

    // Taken from VESC Tool
    constexpr double kp = 0.004;
    constexpr double ki = 0.004;
    constexpr double kd = 0.0001;

    auto node = std::make_shared<arc::MotorController>(kp, ki, kd);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}