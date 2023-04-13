#include "arc_msgs/msg/diff_drive.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>

template <typename T> using Publisher = rclcpp::Publisher<T>;
template <typename T> using Subscriber = rclcpp::Subscription<T>;

using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using DiffDriveMsg = arc_msgs::msg::DiffDrive;

class DifferentialDrive {
  private:
    double wheel_base_;
    double wheel_radius_;

  public:
    DifferentialDrive(double wheel_base, double wheel_radius)
        : wheel_base_{wheel_base}, wheel_radius_{wheel_radius} {}

    // Calculate the left and right wheel velocities based on linear and angular
    // velocity.

    // Constraints:
    // 1) output wheel velocities must be in the range [-1, 1]

    // 2) when linear velocity is 1 and angular velocity is 0, the left and
    // right wheel velocities must be 1

    // 3) when linear velocity is 0 and angular velocity is 1, the left wheel
    // velocity must be 1 and the right wheel velocity must be -1

    // 4) when linear velocity is 0 and angular velocity is -1, the left wheel
    // velocity must be -1 and the right wheel velocity must be 1

    // 5) when linear velocitu is 1 and angular velocity is 1, the left wheel
    // velocity must be 0 and the right wheel velocity must be 1

    // 6) when linear velocity is 1 and angular velocity is -1, the left wheel
    // velocity must be 1 and the right wheel velocity must be 0
    std::pair<double, double>
    calculateWheelVelocities(double linear_velocity,
                             double angular_velocity) const {
        double left_wheel_velocity = 0;
        double right_wheel_velocity = 0;

        // calculate the left and right wheel velocities
        left_wheel_velocity =
            (linear_velocity - (wheel_base_ * angular_velocity) / 2) /
            wheel_radius_;
        right_wheel_velocity =
            (linear_velocity + (wheel_base_ * angular_velocity) / 2) /
            wheel_radius_;

        // normalize the wheel velocities
        double max_wheel_velocity = std::max(std::abs(left_wheel_velocity),
                                             std::abs(right_wheel_velocity));
        if (max_wheel_velocity > 1) {
            left_wheel_velocity /= max_wheel_velocity;
            right_wheel_velocity /= max_wheel_velocity;
        }

        return std::make_pair(left_wheel_velocity * 2,
                              right_wheel_velocity * 2);
    }
};

class TwistToDiffDriveNode : public rclcpp::Node {
  public:
    TwistToDiffDriveNode()
        : Node("twist_to_diff_drive_node"), diff_drive_model_(0.224, 0.15) {
        // Create a DifferentialDrive object with a wheel base of 224mm and a
        // wheel radius of 150mm

        twist_sub_ = this->create_subscription<TwistStampedMsg>(
            "cmd_vel", 10,
            std::bind(&TwistToDiffDriveNode::twist_cb, this,
                      std::placeholders::_1));
        diff_drive_pub_ =
            this->create_publisher<DiffDriveMsg>("diff_drive", 10);
    }

  private:
    Subscriber<TwistStampedMsg>::SharedPtr twist_sub_;
    Publisher<DiffDriveMsg>::SharedPtr diff_drive_pub_;
    DifferentialDrive diff_drive_model_;

    void twist_cb(const TwistStampedMsg::SharedPtr msg) const {
        // Calculate the left and right wheel velocities
        double x = msg->twist.linear.x;
        double z = msg->twist.angular.z;
        auto pair = diff_drive_model_.calculateWheelVelocities(x, z);
        double left_motor = pair.first;
        double right_motor = pair.second;

        // Publish the left and right motor velocities
        auto diff_drive_msg = DiffDriveMsg();
        diff_drive_msg.left_motor = left_motor;
        diff_drive_msg.right_motor = right_motor;
        diff_drive_pub_->publish(diff_drive_msg);
    }
};

int main(int argc, char **argv) {
    // printf("Starting twist_to_diff_drive_node");
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TwistToDiffDriveNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
