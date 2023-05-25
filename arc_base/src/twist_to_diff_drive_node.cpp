#include "arc_base/utils.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include <cmath>
#include <cstdio>
#include <iostream>

namespace arc {

using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using Float64Msg = std_msgs::msg::Float64;

constexpr double TO_RPM = 900; // todo explain

enum class MotorSide { LEFT, RIGHT };

class DifferentialDrive {
  private:
    double wheel_base_;
    double wheel_radius_;
    double gearing_;
    double target_velocity_; // m/s

  public:
    DifferentialDrive(double wheel_base, double wheel_radius, double gearing, double target_velocity)
        : wheel_base_{wheel_base}, wheel_radius_{wheel_radius}, target_velocity_(target_velocity) {

        // target_velocity_ = declare_and_get_parameter<double>(*this, "target_velocity");
    }

    // Calculate the relations between the left and right wheel velocities in
    // [-1, 1]
    std::pair<double, double> calculate_wheel_relations(double linear_velocity,
                                                        double angular_velocity) const {

        double wheel_left = linear_velocity - angular_velocity;
        double wheel_right = linear_velocity + angular_velocity;

        // constrain the wheel velocities to [-1, 1]
        double max_wheel_relation = std::max(std::abs(wheel_left), std::abs(wheel_right));
        if (max_wheel_relation > 1) {
            wheel_left /= max_wheel_relation;
            wheel_right /= max_wheel_relation;
        }

        return std::make_pair(wheel_left, wheel_right);
    }

    // Given the target velocity, calculate the left and right wheel velocities
    // based on the wheel base and wheel radius, given the relations between the
    // left and right wheel velocities in [-1, 1], from the
    // calculate_wheel_relations function
    std::pair<double, double> calculate_wheel_velocities(double wheel_left_relation,
                                                         double wheel_right_relation) const {

        // double wheel_left_velocity =
        //     this->target_velocity_ * wheel_left_relation / (wheel_radius_ * gearing_);
        // double wheel_right_velocity =
        //     this->target_velocity_ * wheel_right_relation / (wheel_radius_ * gearing_);

        double wheel_left_velocity = target_velocity_ * wheel_left_relation;
        double wheel_right_velocity = target_velocity_ * wheel_right_relation;

        return std::make_pair(wheel_left_velocity, wheel_right_velocity);
    }
};

class TwistToDiffDriveNode : public rclcpp::Node {
  public:
    TwistToDiffDriveNode()
        : Node("twist_to_diff_drive_node"),
         target_velocity_{declare_and_get_parameter<double>(*this, "target_velocity")},
         diff_drive_model_(0.224, 0.15, 0.25, target_velocity_) {
        // Create a DifferentialDrive object with a wheel base of 224mm and a
        // wheel radius of 150mm

        // controller rate in Hz
        controller_rate_ = declare_and_get_parameter<double>(*this, "controller_rate");

        // twist command subscriber
        twist_sub_ = this->create_subscription<TwistStampedMsg>(
            "cmd_vel", 10, std::bind(&TwistToDiffDriveNode::twist_cb, this, _1));

        // motor target speed publishers
        motor_speed_left_pub_ = this->create_publisher<Float64Msg>("motor_right/target/motor/speed",
                                                                   rclcpp::QoS(10).reliable());
        motor_speed_right_pub_ = this->create_publisher<Float64Msg>("motor_left/target/motor/speed",
                                                                    rclcpp::QoS(10).reliable());
    }

    void spin() {
        // set ros node rate
        ros2::Rate rate(controller_rate_);

        while (rclcpp::ok()) {
            // publish the left and right motor velocities
            if (publish_motor_speed(MotorSide::LEFT, left_motor_speed_)) {
                // RCLCPP_INFO(this->get_logger(), "Published left motor speed:
                // %0.5f", left_motor_speed_);
            }
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();

            if (publish_motor_speed(MotorSide::RIGHT, right_motor_speed_)) {
                // RCLCPP_INFO(this->get_logger(), "Published right motor speed:
                // %0.5f", right_motor_speed_);
            }
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

  private:
    // rclcpp::Rate rate_;
    Subscriber<TwistStampedMsg>::SharedPtr twist_sub_;
    Publisher<Float64Msg>::SharedPtr motor_speed_left_pub_;
    Publisher<Float64Msg>::SharedPtr motor_speed_right_pub_;

    double controller_rate_;
    double left_motor_speed_;
    double right_motor_speed_;
    double target_velocity_;
    DifferentialDrive diff_drive_model_;

    void twist_cb(const TwistStampedMsg::SharedPtr msg) {
        // RCLCPP_INFO(this->get_logger(), "Received twist:\ntwist.linear.x =
        // %.5f\ntwist.angular.z = %.5f", msg->twist.linear.x,
        // msg->twist.angular.z); Calculate the left and right wheel velocities
        double x = msg->twist.linear.x;
        double z = msg->twist.angular.z;
        auto wheel_relations = diff_drive_model_.calculate_wheel_relations(x, z);
        // double left_motor = wheel_relations.first;
        // double right_motor = wheel_relations.second;
        // converting to wheel velocities

        // RCLCPP_INFO(this->get_logger(), "\nwr_right: %0.5f\nwr_left: %0.5f",
        // wheel_relations.first, wheel_relations.second);

        auto wheel_velocities = diff_drive_model_.calculate_wheel_velocities(
            wheel_relations.first, wheel_relations.second);
        // double left_motor = wheel_velocities.first;
        // double right_motor = wheel_velocities.second;
        left_motor_speed_ = wheel_velocities.first;
        right_motor_speed_ = wheel_velocities.second;

        // RCLCPP_INFO(this->get_logger(), "\nmotor_right: %0.5f\nmotor_left:
        // %0.5f", left_motor, right_motor); Publish the left and right motor
        // velocities
        // auto diff_drive_msg = DiffDriveMsg();
        // diff_drive_msg.left_motor = left_motor;
        // diff_drive_msg.right_motor = right_motor;
        // diff_drive_pub_->publish(diff_drive_msg);

        // publish to the motor controllers
        // {
        //     auto left_motor_msg = Float64Msg();
        //     left_motor_msg.data = left_motor;
        //     motor_speed_left_pub_->publish(left_motor_msg);
        // }
        // {
        //     auto right_motor_msg = Float64Msg();
        //     right_motor_msg.data = right_motor;
        //     motor_speed_right_pub_->publish(right_motor_msg);
        // }
    }

    bool publish_motor_speed(MotorSide motor, double speed) const {
        auto motor_msg = Float64Msg();
        motor_msg.data = speed;
        if (motor == MotorSide::LEFT) {
            motor_speed_left_pub_->publish(motor_msg);
        } else if (motor == MotorSide::RIGHT) {
            motor_speed_right_pub_->publish(motor_msg);
        } else {
            return false;
        }
        return true;
    }
};
} // namespace arc

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);

    std::printf("Starting twist_to_diff_drive_node\n");

    // rclcpp::executors::MultiThreadedExecutor executor;

    auto node = std::make_shared<arc::TwistToDiffDriveNode>();
    node->spin();
    // executor.add_node(node);

    // executor.spin();
    // rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
