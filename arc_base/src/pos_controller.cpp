#include "rclcpp/rclcpp.hpp"
#include "tf2/impl/utils.h"

#include "arc_base/utils.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "pid.hpp"

#include <chrono>
#include <functional>
#include <iostream>

using namespace std::chrono_literals;

namespace arc {

using OdometryMsg = nav_msgs::msg::Odometry;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

class PositionController : public ros2::Node {
  public:
    PositionController() : Node("position_controller") {

        // linear pid gains
        this->declare_parameter("kp_linear", 1.0);
        const double kp_linear = this->get_parameter("kp_linear").as_double();
        std::fprintf(stderr, "%s: %.5f\n", magenta("kp_linear").c_str(),
                     kp_linear);
        this->declare_parameter("ki_linear", 0.0);
        const double ki_linear = this->get_parameter("ki_linear").as_double();
        std::fprintf(stderr, "%s: %.5f\n", magenta("ki_linear").c_str(),
                     ki_linear);
        this->declare_parameter("kd_linear", 0.0);
        const double kd_linear = this->get_parameter("kd_linear").as_double();
        std::fprintf(stderr, "%s: %.5f\n", magenta("kd_linear").c_str(),
                     kd_linear);
        // angular pid gains
        this->declare_parameter("kp_angular", 1.0);
        const double kp_angular = this->get_parameter("kp_angular").as_double();
        std::fprintf(stderr, "%s: %.5f\n", magenta("kp_angular").c_str(),
                     kp_angular);
        this->declare_parameter("ki_angular", 0.0);
        const double ki_angular = this->get_parameter("ki_angular").as_double();
        std::fprintf(stderr, "%s: %.5f\n", magenta("ki_angular").c_str(),
                     ki_angular);
        this->declare_parameter("kd_angular", 0.0);
        const double kd_angular = this->get_parameter("kd_angular").as_double();
        std::fprintf(stderr, "%s: %.5f\n", magenta("kd_angular").c_str(),
                     kd_angular);
        // target velocity in m/s
        this->declare_parameter("target_velocity", 0.1);
        target_velocity_ = this->get_parameter("target_velocity").as_double();
        std::fprintf(stderr, "%s: %.5f\n", magenta("target_velocity").c_str(),
                     target_velocity);
        // controller rate in Hz
        this->declare_parameter("controller_rate");
        int controller_rate = this->get_parameter("controller_rate").as_int();
        std::fprintf(stderr, "%s: %d\n", magenta("controller_rate").c_str(),
                     controller_rate);

        pid_linear_ = PIDController{kp_linear, ki_linear, kd_linear};
        pid_angular_ = PIDController{kp_angular, ki_angular, kd_angular};
        // pid_ = PID{this->get_parameter("kp").as_double(),
        //            this->get_parameter("ki").as_double(),
        //            this->get_parameter("kd").as_double()};

        const String odom_topic = "odom";
        odom_sub_ = this->create_subscription<OdometryMsg>(
            odom_topic, 10,
            [this](OdometryMsg::SharedPtr msg) { msg_actual_odom_ = *msg; });

        const String target_topic = "target_odom";
        target_sub_ = this->create_subscription<OdometryMsg>(
            target_topic, 10,
            [this](OdometryMsg::SharedPtr msg) { msg_target_odom_ = *msg; });

        const String twist_topic = "cmd_vel";
        twist_pub_ = this->create_publisher<TwistStampedMsg>(twist_topic, 10);

        // start control loop
        this->create_wall_timer(
            std::chrono::milliseconds(1000 / controller_rate),
            [this]() { step(); });

        // start publish loop
        this->create_wall_timer(
            std::chrono::milliseconds(1000 / controller_rate),
            [this]() { twist_pub_->publish(msg_cmd_vel_); });
    }

    void step() {
        // get actual position from odom
        const actual_pose = msg_actual_odom_.pose.pose;
        // get target position
        const target_pose = msg_target_odom_.pose.pose;

        // xy pose error
        const double error_x = target_pose.position.x - actual_pose.position.x;
        const double error_y = target_pose.position.y - actual_pose.position.y;
        // yaw error
        // get the target yaw as the angle towards the target from the actual
        // pose only when the target is > pos_tolerance_ away
        // otherwise,align the robot with the target's yaw
        if (std::sqrt(std::pow(error_x, 2) + std::pow(error_y, 2)) >
            pos_tolerance_) {
            const double target_yaw = std::atan2(error_y, error_x);
        } else {
            const double target_yaw = tf2::getYaw(target_pose.orientation);
        }
        const double actual_yaw = tf2::getYaw(actual_pose.orientation);
        double error_yaw = target_yaw - actual_yaw;
        // normalise error_yaw to [-pi, pi]
        yaw_error = std::atan2(std::sin(error_yaw), std::cos(error_yaw));

        // get time delta
        const time_now = this->get_clock()->now();
        const auto delta_time = (time_now = time_prev_).nanoseconds() / 1e9;
        // PIDController steps
        const double linear_vel = pid_linear_.step(error_x + error_y, dt);
        const double angular_vel = pid_angular_.step(error_yaw, dt);
        // update time_prev_
        time_prev_ = time_now;
    }

  private:
    double target_velocity_;
    double time_prev_;

    // pid collections
    PIDController pid_linear_;
    PIDController pid_angular_;

    // msgs
    OdometryMsg msg_actual_odom_;
    OdometryMsg msg_target_odom_;
    TwistStampedMsg msg_cmd_vel_;

    // ros
    Subscriber<OdometryMsg>::SharedPtr odom_sub_;
    Subscriber<OdometryMsg>::SharedPtr target_sub_;
    Publisher<TwistStampedMsg>::SharedPtr twist_pub_;
};

} // namespace arc

auto main(int argc, char **argv) -> int {
    using namespace arc;

    ros2::init(argc, argv);

    auto node = std::make_shared<PositionController>();

    ros2::spin(node);
    ros2::shutdown();

    return 0;
}
