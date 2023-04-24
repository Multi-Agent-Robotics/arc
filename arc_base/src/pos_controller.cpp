
#include "rclcpp/rclcpp.hpp"

#include "arc_base/utils.hpp"
#include "arc_msgs/msg/motor_controller_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <chrono>
#include <functional>
#include <iostream>
#include <map>
#include <vector>

using namespace std::chrono_literals;

namespace arc {

using OdometryMsg = nav_msgs::msg::Odometry;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

class PositionController : public ros2::Node {
  public:
    PositionController() : Node("position_controller") {

        this->declare_parameter("kp", 0.0);
        this->declare_parameter("ki", 0.0);
        this->declare_parameter("kd", 0.0);
        this->declare_parameter("target_velocity", 0.1);

        this->declare_parameter("controller_rate"); // Hz
        int controller_rate = this->get_parameter("controller_rate").as_int();

        pid_ = PID{this->get_parameter("kp").as_double(),
                   this->get_parameter("ki").as_double(),
                   this->get_parameter("kd").as_double()};

        target_velocity_ = this->get_parameter("target_velocity").as_double();

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

        this->create_wall_timer(
            std::chrono::milliseconds(1000 / controller_rate),
            [this]() { twist_pub_->publish(msg_cmd_vel_); });
    }

  private:
    double target_velocity_;
    // pid collections
    PID pid_;
    PID control_pid_;
    PID control_pid_prev_;

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