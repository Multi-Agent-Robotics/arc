
#include "rclcpp/rclcpp.hpp"

#include "arc_base/utils.hpp"
#include "arc_msgs/msg/motor_controller_status.hpp"
#include "circular_buffer.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <algorithm>
#include <chrono>
#include <functional>
#include <iostream>
#include <thread>
#include <vector>

// use chrono literals
using namespace std::chrono_literals;

namespace arc {

using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using VescStateStampedMsg = vesc_msgs::msg::VescStateStamped;
using Float64Msg = std_msgs::msg::Float64;
using MotorControllerStatusMsg = arc_msgs::msg::MotorControllerStatus;

class MotorController : public ros2::Node {
  private:
    // pid collections
    PID pid_;
    PID control_pid_;
    // buffer for moving average
    CircularBuffer<double, 20> actual_buffer_;
    // robot parameters
    RobotParameters robot_params_;

    // constraints
    double output_ = 0.0;
    double output_prev_ = 0.0;
    double integral_ = 0.0;

    // control terms
    ros2::Time time_prev_;
    double target_;
    double actual_;
    double error_;
    double error_prev_;

    // msgs
    VescStateStampedMsg vesc_state_;
    Float64Msg msg_control_;
    MotorControllerStatusMsg msg_status_;

    // dynamic functions
    std::function<double(VescStateStampedMsg &)> get_sensor_value_;
    std::function<double(Float64Msg &)> get_target_value_;

    // ros
    ros2::TimerBase::SharedPtr timer_status_;
    ros2::TimerBase::SharedPtr timer_publish_;
    ros2::TimerBase::SharedPtr timer_control_;
    Subscriber<VescStateStampedMsg>::SharedPtr vesc_state_sub_;
    Subscriber<Float64Msg>::SharedPtr diff_drive_sub_;
    Publisher<Float64Msg>::SharedPtr motor_pub_;
    Publisher<MotorControllerStatusMsg>::SharedPtr status_pub_;

    void pub_status_() {
        // update message

        msg_status_.control.target = target_;
        msg_status_.control.actual = actual_;
        msg_status_.control.error = error_;
        msg_status_.pid.p = control_pid_.p;
        msg_status_.pid.i = control_pid_.i;
        msg_status_.pid.d = control_pid_.d;
        msg_status_.output = output_;
        msg_status_.header.stamp = this->get_clock()->now();

        status_pub_->publish(msg_status_);
    }

  public:
    MotorController(const std::string &name)
        : Node(name.c_str()), control_pid_{0.0, 0.0, 0.0}, robot_params_{*this}, error_prev_{0.0},
          msg_control_{}, msg_status_{} {

        time_prev_ = this->get_clock()->now();

        // ROS PARAMETERS
        const auto motor_id = declare_and_get_parameter<String>(*this, "motor_id");
        const auto controller_rate = declare_and_get_parameter<int>(*this, "controller_rate");

        get_sensor_value_ = [](VescStateStampedMsg &msg) -> double { return msg.state.speed; };
        get_target_value_ = [this](Float64Msg &msg) -> double {
            double target_velocity = msg.data;
            // do conversion from linear m/s to eRPM
            double wheel_circumference = robot_params_.wheel_diameter * M_PI;
            double motor_turns_per_meter = 1.0 / wheel_circumference * robot_params_.gear_ratio;
            double rpm = target_velocity * motor_turns_per_meter * 60.0;
            double erpm = rpm * robot_params_.motor_pole_pairs * 2;
            // std::fprintf(stderr, "wc = %.3f\tmtpm = %.3f\trpm = %.2f\terpm = %.0f", wheel_circumference, motor_turns_per_meter, rpm, erpm);
            // RCLCPP_INFO(this->get_logger(), "wc = %.3f\tmtpm = %.3f\trpm = %.2f\terpm = %.0f", wheel_circumference, motor_turns_per_meter, rpm, erpm);

            return erpm;
        };

        // pid values from param
        pid_.p = declare_and_get_parameter<double>(*this, "pid.kp");
        pid_.i = declare_and_get_parameter<double>(*this, "pid.ki");
        pid_.d = declare_and_get_parameter<double>(*this, "pid.kd");

        const String motor_prefix = "motor_" + motor_id;

        // subscribers
        const String diff_drive_topic = format_topic_path(motor_prefix, "target", "motor", "speed");

        RCLCPP_INFO(this->get_logger(), "diff_drive_topic: " + green(diff_drive_topic));

        diff_drive_sub_ = this->create_subscription<Float64Msg>(
            diff_drive_topic.c_str(), 10,
            [this](Float64Msg::UniquePtr msg) { this->target_ = this->get_target_value_(*msg); });

        const String state_topic = format_topic_path(motor_prefix, "sensors", "core");
        RCLCPP_INFO(this->get_logger(), "state_topic: " + green(state_topic));

        vesc_state_sub_ = this->create_subscription<VescStateStampedMsg>(
            state_topic.c_str(), 10, [this](VescStateStampedMsg::UniquePtr msg) {
                this->vesc_state_ = *msg;
                actual_buffer_.push_back(this->get_sensor_value_(*msg));
                actual_ = actual_buffer_.mean();
            });

        // publishers
        const String motor_topic = format_topic_path(motor_prefix, "commands", "motor", "current");
        RCLCPP_INFO(this->get_logger(), "motor_topic: " + green(motor_topic));
        motor_pub_ = this->create_publisher<Float64Msg>(motor_topic, ros2::QoS(10).reliable());

        const String status_topic = format_topic_path(motor_prefix, "controller", "status");
        RCLCPP_INFO(this->get_logger(), "status_topic: " + green(status_topic));
        status_pub_ = this->create_publisher<MotorControllerStatusMsg>(status_topic,
                                                                       ros2::QoS(10).reliable());

        // controller rate timer
        const auto cooldown = std::chrono::milliseconds(1000 / controller_rate);
        // perform a control step at the controller rate
        timer_control_ = this->create_wall_timer(cooldown, [this]() { step(); });

        // publish control and status at the controller rate
        timer_publish_ = this->create_wall_timer(cooldown, [this]() {
            msg_control_.data = output_;
            motor_pub_->publish(msg_control_);
        });

        timer_status_ = this->create_wall_timer(cooldown, [this]() { pub_status_(); });
    }

    void step() {
        auto time_now = this->get_clock()->now();
        auto time_delta = (time_now - time_prev_).nanoseconds() * 1e-9;

        error_ = target_ - actual_;

        if (std::abs(target_) < 0.1) {
            // reset integral
            integral_ = 0.0;
            if (std::abs(actual_) < 10.0) {
                // reset error
                error_ = 0.0;
            }
        }

        // controller pid output terms
        control_pid_.p = pid_.p * error_; // does not depend on time
        integral_ += error_ * time_delta;
        control_pid_.i = pid_.i * integral_;
        control_pid_.d = pid_.d * (error_prev_ - error_) / time_delta;

        output_ = control_pid_.p + control_pid_.i + control_pid_.d;

        // update previous values
        output_prev_ = output_;
        error_prev_ = error_;
        time_prev_ = time_now;
    }
};
} // namespace arc

auto main(int argc, char **argv) -> int {
    using namespace arc;

    ros2::init(argc, argv);

    auto node = std::make_shared<MotorController>("motor_controller");

    ros2::spin(node);
    ros2::shutdown();

    return 0;
}
