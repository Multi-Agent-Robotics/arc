
#include "rclcpp/rclcpp.hpp"

#include "arc_base/utils.hpp"
#include "arc_msgs/msg/motor_controller_status.hpp"
#include "circular_buffer.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

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
    double max_output_; // rpm | current limit
    double acc_max_;    // m/s²
    double speed_min_;  // rpm
    double output_ = 0.0;
    double output_prev_ = 0.0;
    double integral_ = 0.0;

    // control terms
    ros2::Time time_prev_;
    double target_;
    double actual_;
    double error_;
    // double control_ki_prev_;
    double error_prev_;
    String output_mode_;

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

        status_pub_->publish(msg_status_);
        // RCLCPP_DEBUG(this->get_logger(), "published status");
    }

  public:
    MotorController(const std::string &name)
        : Node(name.c_str()), control_pid_{0.0, 0.0, 0.0}, error_prev_{0.0},
          msg_control_{}, msg_status_{}, robot_params_{*this}
    //   robot_params_(reinterpret_cast<ros2::Node &>(this))
    // load all robot parameters
    {

        time_prev_ = this->get_clock()->now();

        // ROS PARAMETERS
        const auto motor_id =
            declare_and_get_parameter<String>(*this, "motor_id");
        const auto controller_rate =
            declare_and_get_parameter<int>(*this, "controller_rate");
        output_mode_ = declare_and_get_parameter<String>(*this, "output_mode");
        acc_max_ = declare_and_get_parameter<double>(*this, "acc_max");
        speed_min_ = declare_and_get_parameter<double>(*this, "speed_min");

        // load all robot parameters
        // robot_params_ = RobotParameters(this);

        if (output_mode_ == "speed") {
            get_sensor_value_ = [](VescStateStampedMsg &msg) -> double {
                return msg.state.speed;
            };
            get_target_value_ = [](Float64Msg &msg) -> double {
                double target_velocity = msg.data;
                // do conversion from m/s to eRPM
                double wheel_circumference = 0.15 * 2 * M_PI;
                double gear_ratio = 1.0 / 4.0;
                double motor_poles = 14.0;
                double erpm_per_sec =
                    target_velocity /
                    (wheel_circumference * gear_ratio * motor_poles / 2.0);

                return erpm_per_sec;
                // return target_velocity;
            };

            // pid values from param
            pid_.p = declare_and_get_parameter<double>(*this, "pid_speed.kp");
            pid_.i = declare_and_get_parameter<double>(*this, "pid_speed.ki");
            pid_.d = declare_and_get_parameter<double>(*this, "pid_speed.kd");

            // max output from param
            max_output_ = declare_and_get_parameter<double>(*this, "speed_max");

        } else if (output_mode_ == "current") {
            get_sensor_value_ = [](VescStateStampedMsg &msg) {
                return msg.state.current_motor;
            };
            get_target_value_ = [](Float64Msg &msg) {
                double target_velocity = msg.data;
                // do conversion from m/s to current (amps)
                double target_current =
                    target_velocity / 10.0; // placeholder conversion factor

                return target_current;
            };

            // pid values from param
            pid_.p = declare_and_get_parameter<double>(*this, "pid_current.kp");
            pid_.i = declare_and_get_parameter<double>(*this, "pid_current.ki");
            pid_.d = declare_and_get_parameter<double>(*this, "pid_current.kd");

            // max output from param
            max_output_ =
                declare_and_get_parameter<double>(*this, "current_max");

        } else {
            RCLCPP_WARN_ONCE(
                this->get_logger(),
                "output_mode_ != \"speed\" | \"current\", output_mode_: %s",
                output_mode_.c_str());
            ros2::shutdown();
        }

        const String motor_prefix = "motor_" + motor_id;

        // subscribers
        const String diff_drive_topic =
            format_topic_path(motor_prefix, "target", "motor", output_mode_);

        RCLCPP_INFO(this->get_logger(),
                    "diff_drive_topic: " + green(diff_drive_topic));

        diff_drive_sub_ = this->create_subscription<Float64Msg>(
            diff_drive_topic.c_str(), 10, [this](Float64Msg::UniquePtr msg) {
                this->target_ = this->get_target_value_(*msg);
            });

        const String state_topic =
            format_topic_path(motor_prefix, "sensors", "core");
        RCLCPP_INFO(this->get_logger(), "state_topic: " + green(state_topic));

        vesc_state_sub_ = this->create_subscription<VescStateStampedMsg>(
            state_topic.c_str(), 10,
            [this](VescStateStampedMsg::UniquePtr msg) {
                this->vesc_state_ = *msg;
                actual_buffer_.push_back(this->get_sensor_value_(*msg));
                actual_ = actual_buffer_.mean();
            });

        // publishers
        const String motor_topic =
            format_topic_path(motor_prefix, "commands", "motor", output_mode_);
        RCLCPP_INFO(this->get_logger(), "motor_topic: " + green(motor_topic));
        motor_pub_ = this->create_publisher<Float64Msg>(
            motor_topic, ros2::QoS(10).reliable());

        const String status_topic =
            format_topic_path(motor_prefix, "controller", "status");
        RCLCPP_INFO(this->get_logger(), "status_topic: " + green(status_topic));
        status_pub_ = this->create_publisher<MotorControllerStatusMsg>(
            status_topic, ros2::QoS(10).reliable());

        // controller rate timer
        const auto cooldown = std::chrono::milliseconds(1000 / controller_rate);
        // perform a control step at the controller rate
        timer_control_ =
            this->create_wall_timer(cooldown, [this]() { step(); });

        // publish control and status at the controller rate
        timer_publish_ = this->create_wall_timer(cooldown, [this]() {
            // update message and clamp to max output
            // double output = control_pid_.p + control_pid_.i + control_pid_.d;
            // msg_control_.data = output > max_output_ ? max_output_ : output;
            msg_control_.data = output_;

            motor_pub_->publish(msg_control_);
        });

        timer_status_ =
            this->create_wall_timer(cooldown, [this]() { pub_status_(); });
    }

    void step() {
        auto time_now = this->get_clock()->now();
        auto delta_time = (time_now - time_prev_).nanoseconds() * 1e-9;
        // error_ = (target_ - actual_) * delta_time; // error over the time
        // step
        error_ = target_ - actual_; // error

        // controller pid output terms
        control_pid_.p = pid_.p * error_; // does not depend on time
        integral_ += error_ * delta_time;
        control_pid_.i = pid_.i * integral_;
        control_pid_.d = pid_.d * (error_prev_ - error_) / delta_time;

        output_ = control_pid_.p + control_pid_.i + control_pid_.d;
        double delta_output = output_ - actual_; // output_prev_;
        // calculate max allowable change in this timeframe
        double delta_acc_max = acc_max_ * delta_time;
        // limit the output to the max allowable change
        if (std::abs(delta_output) > delta_acc_max) {
            // output_ = output_prev_ + std::copysign(delta_acc_max,
            // delta_output);

            // using the actual measurement here to ensure that the delta given
            // is ALWAYS within the max allowable change from the motor's
            // current state
            output_ = actual_ + std::copysign(delta_acc_max, delta_output);
        }

        // The motor will NOT rotate when the output is between [-900, 900]
        // eRPM. Since we need the a measurement of the current eRPM, to compute
        // a difference from the target value, we need to adjust the minimum
        // output value to be greater than 900 or less than -900.
        // if (std::abs(output_) < speed_min_) {
        //     output_ =
        //         output_ == 0 ? 0 : (output_ > 0 ? speed_min_ : -speed_min_);
        // }

        // In (0, 450) we want to clamp to 900
        // In [450, 900) we want to clamp to 0
        if (std::abs(output_) < 450) { // In (0, 450)
            output_ = std::abs(output_) < std::numeric_limits<double>::epsilon()
                          ? 0
                          : std::copysign(speed_min_, output_);
        } else if (std::abs(output_) < 900) { // In [450, 900)
            output_ = 0;
            integral_ = 0;
            error_ = 0;
            // actual_ = 0;
            pub_status_();
            // ros2::sleep_for(500ms);
        }

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

    // rclcpp::NodeOptions options;
    // options.allow_undeclared_parameters(true);
    // options.automatically_declare_parameters_from_overrides(true);

    auto node = std::make_shared<MotorController>("motor_controller");

    ros2::spin(node);
    ros2::shutdown();

    return 0;
}