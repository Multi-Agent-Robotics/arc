
#include "rclcpp/rclcpp.hpp"

#include "arc_base/utils.hpp"
#include "arc_msgs/msg/motor_controller_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <functional>
#include <iostream>
#include <vector>

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
    PID control_pid_prev_;

    // constraints
    double max_output_; // rpm | current limit
    double acc_max_;    // m/s²
    double speed_min_;  // rpm
    double output_limited_ = 0.0;

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

  public:
    MotorController()
        : Node("motor_controller"), control_pid_{0.0, 0.0, 0.0},
          control_pid_prev_{0.0, 0.0, 0.0}, error_prev_{0.0}, msg_control_{},
          msg_status_{} {

        time_prev_ = this->get_clock()->now();

        // ROS PARAMETERS
        this->declare_parameter("motor_id");
        const String motor_id = this->get_parameter("motor_id").as_string();
        std::fprintf(stderr, "%s: %s\n", magenta("motor_id").c_str(),
                     motor_id.c_str());

        // if (motor_id != "left" || motor_id != "right") {
        //     RCLCPP_WARN_ONCE(this->get_logger(),
        //                  "motor_id != 'left' | 'right', motor_id: %s",
        //                  motor_id.c_str());
        //     ros2::shutdown();
        // }

        this->declare_parameter("controller_rate"); // Hz
        int controller_rate = this->get_parameter("controller_rate").as_int();
        std::fprintf(stderr, "%s: %d\n", magenta("controller_rate").c_str(),
                     controller_rate);

        this->declare_parameter("output_mode"); // speed | current
        output_mode_ = this->get_parameter("output_mode").as_string();
        std::fprintf(stderr, "%s: %s\n", magenta("output_mode").c_str(),
                     output_mode_.c_str());

        this->declare_parameter("acc_max"); // m/s²
        acc_max_ = this->get_parameter("acc_max").as_double();
        std::fprintf(stderr, "%s: %f\n", magenta("acc_max").c_str(), acc_max_);

        this->declare_parameter("speed_min"); // m/s²
        speed_min_ = this->get_parameter("speed_min").as_double();
        std::fprintf(stderr, "%s: %f\n", magenta("speed_min").c_str(),
                     speed_min_);

        if (output_mode_ == "speed") {
            get_sensor_value_ = [](VescStateStampedMsg &msg) -> double {
                return msg.state.speed;
            };
            get_target_value_ = [](Float64Msg &msg) -> double {
                double target_velocity = msg.data;
                // do conversion from m/s to eRPM
                // double wheel_circumference = 0.15 * 2 * M_PI;
                // double gear_ratio = 1.0 / 4.0;
                // double motor_poles = 14.0;
                // double erpm_per_sec =
                //     target_velocity /
                //     (wheel_circumference * gear_ratio * motor_poles / 2.0);

                // return erpm_per_sec;
                return target_velocity;
            };

            // pid values from param
            this->declare_parameter("kp_speed");
            pid_.p = this->get_parameter("kp_speed").as_double();
            std::fprintf(stderr, "%s: %f\n", magenta("kp_speed").c_str(),
                         pid_.p);
            this->declare_parameter("ki_speed");
            pid_.i = this->get_parameter("ki_speed").as_double();
            std::fprintf(stderr, "%s: %f\n", magenta("ki_speed").c_str(),
                         pid_.i);

            this->declare_parameter("kd_speed");
            pid_.d = this->get_parameter("kd_speed").as_double();
            std::fprintf(stderr, "%s: %f\n", magenta("kd_speed").c_str(),
                         pid_.d);

            // max output from param
            this->declare_parameter("speed_max");
            max_output_ = this->get_parameter("speed_max").as_double();
            std::fprintf(stderr, "%s: %f\n", magenta("speed_max").c_str(),
                         max_output_);
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
            this->declare_parameter("kp_current");
            pid_.p = this->get_parameter("kp_current").as_double();
            this->declare_parameter("ki_current");
            pid_.i = this->get_parameter("ki_current").as_double();
            this->declare_parameter("kd_current");
            pid_.d = this->get_parameter("kd_current").as_double();

            // max output from param
            this->declare_parameter("current_max");
            max_output_ = this->get_parameter("current_max").as_double();
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
                this->actual_ = this->get_sensor_value_(vesc_state_);
            });

        const String state_topic =
            format_topic_path(motor_prefix, "sensors", "core");
        RCLCPP_INFO(this->get_logger(), "state_topic: " + green(state_topic));

        vesc_state_sub_ = this->create_subscription<VescStateStampedMsg>(
            state_topic.c_str(), 10,
            [this](VescStateStampedMsg::UniquePtr msg) {
                this->vesc_state_ = *msg;
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
            msg_control_.data = output_limited_;

            motor_pub_->publish(msg_control_);
        });

        timer_status_ = this->create_wall_timer(cooldown, [this]() {
            // update message

            msg_status_.control.target = target_;
            msg_status_.control.actual = actual_;
            msg_status_.control.error = error_;
            msg_status_.pid.p = control_pid_.p;
            msg_status_.pid.i = control_pid_.i;
            msg_status_.pid.d = control_pid_.d;
            msg_status_.output = output_limited_;

            status_pub_->publish(msg_status_);
            // RCLCPP_DEBUG(this->get_logger(), "published status");
        });
    }

    void step() {
        auto time_now = this->get_clock()->now();
        auto delta_time = (time_now - time_prev_).nanoseconds() * 1e-9;
        error_ = (target_ - actual_) * delta_time; // error over the time step

        // controller pid output terms
        control_pid_.p = pid_.p * error_;
        control_pid_.i = pid_.i * error_ + control_pid_prev_.i;
        control_pid_.d = pid_.d * (error_prev_ - error_);

        // calculate max allowable change in this timeframe
        double delta_acc_max = acc_max_ * delta_time;
        // u_limited(t) = u(t-1) + max(-Δu_max, min(Δu_max, u(t) - u(t-1)))
        double output = control_pid_.p + control_pid_.i + control_pid_.d;
        double output_prev =
            control_pid_prev_.p + control_pid_prev_.i + control_pid_prev_.d;
        output_limited_ =
            actual_ + output +
            std::max(-delta_acc_max,
                     std::min(delta_acc_max, output - output_prev));

        // The motor will NOT rotate when the output is between [-900, 900]
        // eRPM. Since we need the a measurement of the current eRPM, to compute
        // a difference from the target value, we need to adjust the minimum
        // output value to be greater than 900 or less than -900.
        if (std::abs(output_limited_) < speed_min_) {
            output_limited_ =
                output_limited_ == 0
                    ? 0
                    : (output_limited_ > 0 ? speed_min_ : -speed_min_);
        }

        // update previous values
        control_pid_prev_ = control_pid_;
        // control_pid_prev_.p = control_pid_.p;
        // control_pid_prev_.i = control_pid_.i;
        // control_pid_prev_.d = control_pid_.d;

        error_prev_ = error_;
        time_prev_ = time_now;
    }
};
} // namespace arc

auto main(int argc, char **argv) -> int {
    using namespace arc;

    ros2::init(argc, argv);

    auto node = std::make_shared<MotorController>();

    ros2::spin(node);
    ros2::shutdown();

    return 0;
}