#include <chrono>
#include <memory>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"

class PIDController : public rclcpp::Node
{
public:
  PIDController() : Node("pid_controller")
  {
    // ...
    target_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "target", 10, std::bind(&PIDController::target_callback, this, std::placeholders::_1));
    state_sub_ = this->create_subscription<std_msgs::msg::Float64>(
      "state", 10, std::bind(&PIDController::state_callback, this, std::placeholders::_1));
    control_pub_ = this->create_publisher<std_msgs::msg::Float64>("control", 10);

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1),
      std::bind(&PIDController::control_loop, this));

    // Initialize target current with 0.0
    target_current_ = 0.0;
    prev_time_ = this->now();
  }

private:
  void target_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    // Convert target speed in m/s to target current
    double target_speed_m_s = msg->data;
    target_current_ = speed_to_current(target_speed_m_s);
  }

  void state_callback(const std_msgs::msg::Float64::SharedPtr msg)
  {
    state_ = msg->data;
  }

  void control_loop()
  {
  {
    rclcpp::Time current_time = this->now();
    double dt = (current_time - prev_time_).seconds();

    double error = target_ - state_;
    double p_term = kp_ * error;
    
    integral_ += error * dt; // Multiply by dt for proper integration
    double i_term = ki_ * integral_;
    
    double d_term = kd_ * (error - prev_error_) / dt; // Divide by dt for proper differentiation

    double control = p_term + i_term + d_term;

    // Limit the change in control command (eRPM or current)
    double delta_control = control - prev_control_;
    double max_delta_control = max_acceleration_ * dt; // Replace max_acceleration_ with an appropriate value
    if (std::abs(delta_control) > max_delta_control)
    {
      control = prev_control_ + std::copysign(max_delta_control, delta_control);
    }

    auto control_msg = std_msgs::msg::Float64();
    control_msg.data = control;
    control_pub_->publish(control_msg);

    prev_error_ = error;
    prev_time_ = current_time;
  }

  double speed_to_current(double speed)
  {
    double target_wheel_angular_speed = speed / wheel_radius_;
    double target_motor_angular_speed = target_wheel_angular_speed * gear_ratio_;
    double target_motor_speed_RPM = target_motor_angular_speed * (60 / (2 * M_PI));
    double target_motor_voltage = target_motor_speed_RPM / kv_;
    
    double required_torque = mass_ * g_ * friction_coefficient_ * wheel_radius_ / (2 * gear_ratio_);
    double target_motor_current = required_torque / kt_;
    double resistance_current = target_motor_voltage / winding_resistance_;

    double total_target_current = target_motor_current + resistance_current;
    return total_target_current;
  }

  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr target_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr control_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Time prev_time_;

  double target_{0.0};
  double state_{0.0};
  double prev_error_{0.0};
  double integral_{0.0};

  const double kp_{1.0}; // Proportional gain
  const double ki_{0.0}; // Integral gain
  const double kd_{0.0}; // Derivative gain

  double target_current_;

  // Robot parameters
  const double wheel_radius_ = 0.15; // Wheel radius in meters
  const double gear_ratio_ = 60.0 / 15.0; // Gear ratio: wheel pulley teeth / motor pulley teeth
  const double mass_ = /* Your robot's mass in kg */;
  const double g_ = 9.81; // Acceleration due to gravity in m/s²
  const double friction_coefficient_ = /* Choose a value between 0.01 and 0.02 */;

  // Motor parameters
  const double kv_ = /* Your motor's Kv rating (RPM per volt) */;
  const double kt_ = /* Your motor's torque constant */;
  const double winding_resistance_ = /* Your motor's winding resistance (ohms) */;

  double prev_control_{0.0};
  const double max_acceleration_{/* Your desired max acceleration value */};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PIDController>());
  rclcpp::shutdown();
  return 0;
}
