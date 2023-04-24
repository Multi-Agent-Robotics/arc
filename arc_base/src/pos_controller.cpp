
#include "rclcpp/rclcpp.hpp"

#include "arc_base/utils.hpp"
#include "arc_msgs/msg/motor_controller_status.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <functional>
#include <iostream>
#include <vector>

namespace arc {

using OdometryMsg = nav_msgs::msg::Odometry;
using TwistStampedMsg = geometry_msgs::msg::TwistStamped;

class PositionController : ros2::Node {
  public:
    PositionController() : Node("position_controller"), {

        this->declare_parameters("position_controller",
                                 {
                                     {"p", 0.0},
                                     {"i", 0.0},
                                     {"d", 0.0},
                                     {"max_output", 0.0},
                                     {"acc_max", 0.0},
                                     {"output_mode", "rpm"},
                                 });
    }

  private:
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
    Subcriber<OdometryMsg>::SharedPtr target_sub_;
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