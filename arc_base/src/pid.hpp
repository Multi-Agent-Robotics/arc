#include "rclcpp/rclcpp.hpp"

#include "arc_base/utils.hpp"

#include <iostream>

namespace arc {

class PIDController {
  public:
    PIDController(double kp, double ki, double kd)
        : kp_(kp), ki_(ki), kd_(kd) {}

    double compute(double error, double dt) {
        // double error = target - current;                // proportional
        integral_ += error * dt;                        // integral
        double derivative = (error - prev_error_) / dt; // derivative

        prev_error_ = error;

        return kp_ * error + ki_ * integral_ + kd_ * derivative;
    }

  private:
    double kp_;
    double ki_;
    double kd_;

    double prev_error_;
    double integral_;
};
} // namespace arc
