#include <functional>

#include "ros2/ros2.hpp"

#include "arc_msgs/msg/motor_controller_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

#include <functional>
#include <iostream>
#include <vector>

template <typename T> using Vec = std::vector<T>;

using String = std::string;

// A utility function that takes a variadic number of strings
// and concatenates them into a single string formatted as a ros2 topic path
template <typename... Args> String format_topic_path(Args... args) {
    Vec<String> topic_path = {args...};
    String topic_path_str = "";
    for (auto &topic : topic_path) {
        topic_path_str += topic + "/";
    }
    return topic_path_str;
}

namespace arc {

// pid control struct
struct PID {
    double p;
    double i;
    double d;
};

using namespace ros2 = rclcpp;

using namespace std::placeholders;

template <typename T> using Publisher = ros2::Publisher<T>;
template <typename T> using Subscriber = ros2::Subscription<T>;

using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using VescStateStampedMsg = vesc_msgs::msg::VescStateStamped;
using Float64Msg = std_msgs::msg::Float64;
using MotorControllerStatusMsg = arc_msgs::msg::MotorControllerStatus;

class MotorController : public ros2::Node {
  private:
    // double kp_;
    // double ki_;
    // double kd_;
    PID pid_;
    PID control_pid_;
    double max_output_;
    ros2::Time time_prev_;
    double target_;
    double actual_;
    double error_;
    double control_ki_prev_;
    double error_prev_;
    VescStateStampedMsg vesc_state_;
    String output_mode_;

    Float64Msg msg_control_;
    MotorControllerStatusMsg msg_status_;

    std::function<double(VescStateStampedMsg::SharedPtr)> get_sensor_value_;
    std::function<double(Float64Msg::SharedPtr)> get_target_value_;

    // ros
    ros2::TimerBase::SharedPtr timer_status_;
    ros2::TimerBase::SharedPtr timer_control_;
    Subscriber<VescStateStampedMsg>::SharedPtr vesc_state_sub_;
    Subscriber<Float64Msg>::SharedPtr diff_drive_sub_;
    Publisher<Float64Msg>::SharedPtr motor_pub_;
    Publisher<MotorControllerStatusMsg>::SharedPtr status_pub_;

  public:
    MotorController()
        : Node("motor_controller"), control_ki_prev_{0.0},
          error_prev_{0.0}, msg_control_{}, msg_status_{} {

        time_prev_ = this->get_clock()->now();

        // ROS PARAMETERS
        this->declare_parameter("motor_id");
        const String motor_id = this->get_parameter("motor_id").as_string();
        if (motor_id != "left" || motor_id != "right") {
            RCLCPP_ERROR(this->get_logger(), "motor_id != \"left\" |
            \"right\", motor_id: %s", motor_id.c_str());
            ros2::shutdown();
        }

        this->declare_parameter("controller_rate");
        int controller_rate = this->get_parameter("controller_rate").as_int();

        this->declare_parameter("output_mode");
        output_mode_ = this->get_parameter("output_mode").as_string();
        // ------------------

        if (output_mode_ == "speed") {
            get_sensor_value_ = [](VescStateStampedMsg::SharedPtr msg) {
                return msg->state.speed;
            };
            get_target_value_ = [](Float64Msg::SharedPtr msg) {
                double target_velocity = msg->data;
                // do conversion from m/s to eRPM
                double wheel_circumference = 0.15 * 2 * M_PI;
                double gear_ratio = 1.0 / 4.0;
                double motor_poles = 14.0;
                double erpm_per_sec = target_velocity / (wheel_circumference *
                                               gear_ratio * motor_poles / 2.0);

                return value;
            };

            // pid values from param
            this->declare_parameter("kp_speed");
            pid_.p = this->get_parameter("kp_speed").as_double();
            this->declare_parameter("ki_speed");
            pid_.i = this->get_parameter("ki_speed").as_double();
            this->declare_parameter("kd_speed");
            pid_.d = this->get_parameter("kd_speed").as_double();

            // max output from param
            this->declare_parameter("speed_max");
            max_output_ = this->get_parameter("speed_max").as_double();

        } else if (output_mode_ == "current") {
            get_sensor_value_ = [](VescStateStampedMsg::SharedPtr msg) {
                return msg->state.current_motor;
            };
            get_target_value_ = [](Float64Msg::SharedPtr msg) {
                double target_velocity = msg->data;
                // do conversion from m/s to current (amps)
                double target_current = target_velocity / 10.0; // placeholder conversion factor

                return value;
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
            RCLCPP_ERROR(
                this->get_logger(),
                "output_mode_ != \"speed\" | \"current\", output_mode_: %s",
                output_mode_.c_str());
            ros2::shutdown();
        }

        const String motor_prefix = "motor_" + motor_id;

        // subscribers
        const String diff_drive_topic =
            format_topic_path(motor_prefix, "target", "motor", output_mode_);
        diff_drive_sub_ = this->create_subscription<Float64Msg>(
            diff_drive_topic, 10,
            [this](Float64Msg::SharedPtr msg) {
                target_ = get_target_value_(msg);
                actual_ = get_sensor_value_(&vesc_state_);
             });

        const String state_topic =
            format_topic_path(motor_prefix, "sensors", "core");
        vesc_state_sub_ = this->create_subscription<VescStateStampedMsg>(
            state_topic, 10,
            [this](VescStateStampedMsg::SharedPtr msg) { vesc_state_ = *msg; });

        // publishers
        const String motor_topic =
            format_topic_path(motor_prefix, "commands", "motor", output_mode_);
        motor_pub_ = this->create_publisher<Float64Msg>(
            motor_topic, ros2::QoS(10).reliable());

        const String status_topic =
            format_topic_path(motor_prefix, "controller", "status");
        status_pub_ = this->create_publisher<MotorControllerStatusMsg>(
            status_topic, ros2::QoS(10).reliable());

        // controller rate timer
        const auto cooldown = std::chrono::milliseconds(1000 / controller_rate);
        // perform a control step at the controller rate
        timer_controller_ = this->create_wall_timer(
            cooldown, [this]() { step(); });

        // publish control and status at the controller rate
        timer_control_ = this->create_wall_timer(
            cooldown, [this]() {
                // update message and clamp to max output
                double output = control_pid_.p + control_pid_.i + control_pid_.d;
                msg_control_.data = output > max_output_ ? max_output_ : output;

                motor_pub_->publish(msg_control_);
            });

        timer_status_ = this->create_wall_timer(
            cooldown, [this]() {
                // update message
                msg_status_.control.target = target_;
                msg_status_.control.actual = actual_;
                msg_status_.control.error = error;
                msg_status_.pid.p = control_pid_.p;
                msg_status_.pid.i = control_pid_.i;
                msg_status_.pid.d = control_pid_.d;

                status_pub_->publish(msg_status_);
            });
    }

    void step() {
        auto time_now = this->get_clock()->now();
        auto delta_time = (time_now - time_prev_).nanoseconds() * 1e-9;
        error_ = target_ - actual_ * delta_time; // error over the time step

        // controller pid output terms
        control_pid_.p = kp_ * error_;
        control_pid_.i = control_ki_prev_ + ki_ * error_;
        control_pid_.d = (error_prev_ - error) * kd_;

        // update previous values
        control_ki_prev_ = control_pid_.i;
        error_prev_ = error_;
        time_prev_ = time_now;
    }
};
} // namespace arc

int main(int argc, char **argv) {
    using namespace arc;

    ros2::init(argc, argv);

    auto node = std::make_shared<arc::MotorController>();

    ros2::spin(node);
    ros2::shutdown();

    return 0;
}