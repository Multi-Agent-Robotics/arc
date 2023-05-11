#include "rclcpp/rclcpp.hpp"
#include <string>

#include "arc_base/utils.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

using VescStateStampedMsg = vesc_msgs::msg::VescStateStamped;
using Float64Msg = std_msgs::msg::Float64;

class RadPerSecNode : public rclcpp::Node {
  public:
    RadPerSecNode(const std::string &node_name) : Node(node_name), _robot_params(*this) {
        const std::string motor_id = declare_and_get_parameter<String>(*this, "motor_id");
        const std::string motor_prefix = "motor_" + motor_id;

        _pub_rad_per_sec = create_publisher<Float64Msg>(
            format_topic_path(motor_prefix, "sensors", "rad_per_sec"), 10);

        _sub_vesc_state = create_subscription<VescStateStampedMsg>(
            format_topic_path(motor_prefix, "sensors", "core"), 10,
            [this](VescStateStampedMsg::UniquePtr msg) {
                _vesc_state = *msg;

                Float64Msg msg_rad_per_sec;
                const double ERPM = _vesc_state.state.speed;
                const double pole_pairs =
                    _robot_params.motor_pole_pairs; // number of pole pairs divided by 2
                msg_rad_per_sec.data = (ERPM / pole_pairs) * (2 * M_PI / 60); // rad/s
                _pub_rad_per_sec->publish(msg_rad_per_sec);
            });
    }

  private:
    RobotParameters _robot_params;
    VescStateStampedMsg _vesc_state;
    Subscriber<VescStateStampedMsg>::SharedPtr _sub_vesc_state;
    Publisher<Float64Msg>::SharedPtr _pub_rad_per_sec;
};

auto main(int argc, char **argv) -> int {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RadPerSecNode>("motor_erpm_to_rad_per_sec_node");
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
