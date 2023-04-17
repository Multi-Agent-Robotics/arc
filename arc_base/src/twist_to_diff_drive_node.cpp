#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <iostream>

namespace arc {

using namespace std::placeholders;

template <typename T> using Publisher = rclcpp::Publisher<T>;
template <typename T> using Subscriber = rclcpp::Subscription<T>;

using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using float64msg = std_msgs::msg::Float64;

constexpr double MOTOR_T = 15.0;
constexpr double WHEEL_T = 60.0;
constexpr double TO_RPM = 900; // todo explain
constexpr double GEARING = (MOTOR_T / WHEEL_T);

class DifferentialDrive {
  private:
    double wheel_base_;
    double wheel_radius_;
    double target_velocity_;
    double gearing_;

  public:
    DifferentialDrive(double wheel_base, double wheel_radius, double gearing,
                      double target_velocity)
        : wheel_base_{wheel_base}, wheel_radius_{wheel_radius},
          target_velocity_{target_velocity}, gearing_{gearing} {}

    // Calculate the relations between the left and right wheel velocities in
    // [-1, 1]
    std::pair<double, double>
    calculate_wheel_relations(double linear_velocity,
                              double angular_velocity) const {

        double wheel_left = linear_velocity - angular_velocity;
        double wheel_right = linear_velocity + angular_velocity;

        // constrain the wheel velocities to [-1, 1]
        double max_wheel_velocity =
            std::max(std::abs(wheel_left), std::abs(wheel_right));
        if (max_wheel_velocity > 1) {
            wheel_left /= max_wheel_velocity;
            wheel_right /= max_wheel_velocity;
        }

        return std::make_pair(wheel_left, wheel_right);
    }

    // Given the target velocity, calculate the left and right wheel velocities
    // based on the wheel base and wheel radius, given the relations between the
    // left and right wheel velocities in [-1, 1], from the
    // calculate_wheel_relations function
    std::pair<double, double>
    calculate_wheel_velocities(double wheel_left_relation,
                               double wheel_right_relation) const {

        double wheel_left_velocity = this->target_velocity_ *
                                     wheel_left_relation /
                                     (wheel_radius_ * gearing_) * TO_RPM;
        double wheel_right_velocity = this->target_velocity_ *
                                      wheel_right_relation /
                                      (wheel_radius_ * gearing_) * TO_RPM;

        return std::make_pair(wheel_left_velocity, wheel_right_velocity);
    }
};

class TwistToDiffDriveNode : public rclcpp::Node {
  public:
    TwistToDiffDriveNode()
        : Node("twist_to_diff_drive_node"),
          diff_drive_model_(0.224, 0.15, GEARING, 1) {
        // Create a DifferentialDrive object with a wheel base of 224mm and a
        // wheel radius of 150mm

        twist_sub_ = this->create_subscription<TwistStampedMsg>(
            "cmd_vel", 10,
            std::bind(&TwistToDiffDriveNode::twist_cb, this, _1));

        motor_speed_left_pub_ =
            this->create_publisher<float64msg>("motor_right/target/motor/speed", 10);
        motor_speed_left_pub_ =
            this->create_publisher<float64msg>("motor_left/target/motor/speed", 10);
    }

  private:
    Subscriber<TwistStampedMsg>::SharedPtr twist_sub_;
    Publisher<float64msg>::SharedPtr motor_speed_left_pub_;
    Publisher<float64msg>::SharedPtr motor_speed_right_pub_;
    DifferentialDrive diff_drive_model_;

    void twist_cb(const TwistStampedMsg::SharedPtr msg) const {
        // Calculate the left and right wheel velocities
        double x = msg->twist.linear.x;
        double z = msg->twist.angular.z;
        auto wheel_relations =
            diff_drive_model_.calculate_wheel_relations(x, z);
        // double left_motor = wheel_relations.first;
        // double right_motor = wheel_relations.second;
        // converting to wheel velocities

        // RCLCPP_INFO(this->get_logger(), "\nwr_right: %0.5f\nwr_left: %0.5f",
        // wheel_relations.first, wheel_relations.second);

        auto wheel_velocities = diff_drive_model_.calculate_wheel_velocities(
            wheel_relations.first, wheel_relations.second);
        double left_motor = wheel_velocities.first;
        double right_motor = wheel_velocities.second;

        // RCLCPP_INFO(this->get_logger(), "\nmotor_right: %0.5f\nmotor_left:
        // %0.5f", left_motor, right_motor); Publish the left and right motor
        // velocities
        // auto diff_drive_msg = DiffDriveMsg();
        // diff_drive_msg.left_motor = left_motor;
        // diff_drive_msg.right_motor = right_motor;
        // diff_drive_pub_->publish(diff_drive_msg);

        // publish to the motor controllers
        auto left_motor_msg = float64msg();
        left_motor_msg.data = left_motor;
        motor_speed_left_pub_->publish(left_motor_msg);

        auto right_motor_msg = float64msg();
        right_motor_msg.data = right_motor;
        motor_speed_right_pub_->publish(right_motor_msg);
    }
};
} // namespace arc

int main(int argc, char **argv) {
    printf("Starting twist_to_diff_drive_node");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<arc::TwistToDiffDriveNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
