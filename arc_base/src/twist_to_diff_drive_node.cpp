#include "arc_msgs/msg/diff_drive.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <iostream>

template <typename T> using Publisher = rclcpp::Publisher<T>;

template <typename T> using Subscriber = rclcpp::Subscription<T>;

using TwistMsg = geometry_msgs::msg::Twist;
using DiffDriveMsg = arc_msgs::msg::DiffDrive;

class DifferentialDrive {
  private:
    double wheel_base_;
    double wheel_radius_;

  public:
    DifferentialDrive(double wheel_base, double wheel_radius)
        : wheel_base_{wheel_base}, wheel_radius_{wheel_radius} {}

    // Calculate the left and right wheel velocities based on linear and angular
    // velocity
    auto calculateWheelVelocities(double linear_vel, double angular_vel) const {
        return std::pair{
            (linear_vel - (angular_vel * wheel_base_ / 2.0)) / wheel_radius_,
            (linear_vel + (angular_vel * wheel_base_ / 2.0)) / wheel_radius_};
    }
};

class TwistToDiffDriveNode : public rclcpp::Node {
  public:
    TwistToDiffDriveNode() : Node("twist_to_diff_drive_node") {
        // Create a DifferentialDrive object with a wheel base of 224mm and a
        // wheel radius of 150mm
        diff_drive_model_ = DifferentialDrive(0.224, 0.15);

        twist_sub_ = this->create_subscription<TwistMsg>(
            "cmd_vel", 10,
            std::bind(&TwistToDiffDriveNode::twist_cb, this,
                      std::placeholders::_1));
        diff_drive_pub_ =
            this->create_publisher<DiffDriveMsg>("diff_drive", 10);
    }

  private:
    Subscriber<TwistMsg>::SharedPtr twist_sub_;
    Publisher<DiffDriveMsg>::SharedPtr diff_drive_pub_;
    DifferentialDrive diff_drive_model_;
    void twist_cb(TwistMsg msg) {
        // Calculate the left and right wheel velocities
        auto [left_motor, right_motor] =
            diff_drive_model_.calculateWheelVelocities(msg.linear.x,
                                                       msg.angular.z);

        // Publish the left and right motor velocities
        auto diff_drive_msg = DiffDriveMsg();
        diff_drive_msg.left_motor = left_motor;
        diff_drive_msg.right_motor = right_motor;
        diff_drive_pub_->publish(diff_drive_msg);
    }
};

int main() {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<TwistToDiffDriveNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
