#include "rclcpp/rclcpp.hpp"
#include "tf2/impl/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"

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
    PositionController()
        : Node("position_controller"), pid_linear_{0.0, 0.0, 0.0}, pid_angular_{
                                                                       0.0, 0.0,
                                                                       0.0} {
        // linear pid gains
        const double kp_linear =
            declare_and_get_parameter<double>(*this, "pid_linear.kp");
        const double ki_linear =
            declare_and_get_parameter<double>(*this, "pid_linear.ki");
        const double kd_linear =
            declare_and_get_parameter<double>(*this, "pid_linear.kd");
        // angular pid gains
        const double kp_angular =
            declare_and_get_parameter<double>(*this, "pid_angular.kp");
        const double ki_angular =
            declare_and_get_parameter<double>(*this, "pid_angular.ki");
        const double kd_angular =
            declare_and_get_parameter<double>(*this, "pid_angular.kd");
        // target velocity in m/s
        target_velocity_ =
            declare_and_get_parameter<double>(*this, "target_velocity");
        pos_tolerance_ =
            declare_and_get_parameter<double>(*this, "pos_tolerance");
        // controller rate in Hz
        const int controller_rate =
            declare_and_get_parameter<int>(*this, "controller_rate");

        pid_linear_ = PIDController{kp_linear, ki_linear, kd_linear};
        pid_angular_ = PIDController{kp_angular, ki_angular, kd_angular};

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
        const auto actual_pose = msg_actual_odom_.pose.pose;
        // get target position
        const auto target_pose = msg_target_odom_.pose.pose;

        // xy pose error
        const double error_x = target_pose.position.x - actual_pose.position.x;
        const double error_y = target_pose.position.y - actual_pose.position.y;
        // yaw error
        // get the target yaw as the angle towards the target from the actual
        // pose only when the target is > pos_tolerance_ away
        // otherwise,align the robot with the target's yaw
        double target_yaw = 0.0;
        if (std::sqrt(std::pow(error_x, 2) + std::pow(error_y, 2)) >
            pos_tolerance_) {
            target_yaw = std::atan2(error_y, error_x);
        } else {
            auto q = tf2::Quaternion();
            tf2::fromMsg(target_pose.orientation, q);
            target_yaw = tf2::impl::getYaw(q);
        }

        auto q = tf2::Quaternion();
        tf2::fromMsg(actual_pose.orientation, q);
        const double actual_yaw = tf2::impl::getYaw(q);
        double error_yaw = target_yaw - actual_yaw;
        // normalise error_yaw to [-pi, pi]
        error_yaw = std::atan2(std::sin(error_yaw), std::cos(error_yaw));

        // get time delta
        const auto time_now = this->get_clock()->now();

        const auto dt = (time_now - time_prev_).nanoseconds() * 1e-9;
        // PIDController steps
        const double linear_vel = pid_linear_.compute(error_x + error_y, dt);
        const double angular_vel = pid_angular_.compute(error_yaw, dt);
        // update time_prev_
        time_prev_ = time_now;
    }

  private:
    double target_velocity_;
    double pos_tolerance_;
    ros2::Time time_prev_;

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
