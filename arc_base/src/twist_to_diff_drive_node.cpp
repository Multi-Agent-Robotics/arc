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
    double target_velocity_;

  public:
    DifferentialDrive(double wheel_base, double wheel_radius,
                      double target_velocity)
        : wheel_base_{wheel_base}, wheel_radius_{wheel_radius},
          target_velocity_{target_velocity} {}

    // Calculate the relations between the left and right wheel velocities in
    // [-1, 1]
    std::pair<double, double>
    calculate_wheel_relations(double linear_velocity,
                              double angular_velocity) const {

        double left_wheel = linear_velocity - angular_velocity;
        double right_wheel = linear_velocity + angular_velocity;

        // // constrain the wheel velocities to [-1, 1]
        // double max_wheel_velocity =
        //     std::max(std::abs(left_wheel), std::abs(right_wheel));
        // if (max_wheel_velocity > 1) {
        //     left_wheel /= max_wheel_velocity;
        //     right_wheel /= max_wheel_velocity;
        // }

        return std::make_pair(left_wheel, right_wheel);
    }

    // Given the target velocity, calculate the left and right wheel velocities
    // based on the wheel base and wheel radius, given the relations between the
    // left and right wheel velocities in [-1, 1], from the
    // calculate_wheel_relations function
    std::pair<double, double>
    calculate_wheel_velocities(double target_velocity,
                               double left_wheel_relation,
                               double right_wheel_relation) const {

        double left_wheel_velocity =
            target_velocity * left_wheel_relation / wheel_radius_;
        double right_wheel_velocity =
            target_velocity * right_wheel_relation / wheel_radius_;

        return std::make_pair(left_wheel_velocity, right_wheel_velocity);
    }
};

class TwistToDiffDriveNode : public rclcpp::Node {
  public:
    TwistToDiffDriveNode()
        : Node("twist_to_diff_drive_node"), diff_drive_model_(0.224, 0.15, 1) {
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
        auto wheel_relations =
            diff_drive_model_.calculate_wheel_relations(x, z);
        double left_motor = wheel_relations.first;
        double right_motor = wheel_relations.second;
        // auto wheel_velocities = diff_drive_model_.calculate_wheel_velocities(
        //     0.5, wheel_relations.first, wheel_relations.second);
        // double left_motor = wheel_velocities.first;
        // double right_motor = wheel_velocities.second;

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
