#include <functional>

#include "rclcpp/rclcpp.hpp"

#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_msgs/msg/float64.hpp"
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

namespace arc {

using namespace std::placeholders;

template <typename T> using Publisher = rclcpp::Publisher<T>;
template <typename T> using Subscriber = rclcpp::Subscription<T>;

using TwistStampedMsg = geometry_msgs::msg::TwistStamped;
using VescStateStampedMsg = vesc_msgs::msg::VescStateStamped;
using float64msg = std_msgs::msg::Float64;

enum class MotorPosition { Left, Right };

class MotorController : public rclcpp::Node {
  private:
    double kp_;
    double ki_;
    double kd_;
    rclcpp::Time time_prev_;
    double control_ki_prev_;
    double error_eRPM_prev_;
    // VescStateStampedMsg = vesc_state_;
    double vesc_state_speed_;
    // ros
    Subscriber<VescStateStampedMsg>::SharedPtr vesc_state_sub_;
    Subscriber<float64msg>::SharedPtr diff_drive_sub_;
    Publisher<float64msg>::SharedPtr motor_pub_;

  public:
    MotorController(double kp, double ki, double kd, MotorPosition motor_pos)
        : Node("motor_controller"), kp_{kp}, ki_{ki}, kd_{kd},
          control_ki_prev_{0.0}, error_eRPM_prev_{0.0} {

        time_prev_ = this->get_clock()->now();
        std::string motor_prefix =
            motor_pos == MotorPosition::Left ? "motor_left" : "motor_right";

        std::string diff_drive_topic =
            motor_prefix + std::string("/target/motor/speed");
        diff_drive_sub_ = this->create_subscription<float64msg>(
            diff_drive_topic, 10, std::bind(&MotorController::step, this, std::placeholders::_1));

        std::string motor_topic = motor_prefix + "/commands/motor/speed";
        motor_pub_ = this->create_publisher<float64msg>(motor_topic, 10);

        // std::string state_topic = motor_prefix + "sensors/core";
        // vesc_state_sub_ = this->create_subscription<VescStateStampedMsg>(
        //     state_topic, 10,
        //     std::bind(&MotorController::vesc_state_cb, this, _1));
    }

    // void vesc_state_cb(const VescStateStampedMsg::SharedPtr msg) {
    //     vesc_state_speed_ = msg->state.speed;
    // }

    void step(const float64msg::SharedPtr msg) {
        double target_eRPM = msg->data;
        double current_eRPM = vesc_state_speed_;

        auto time_now = this->get_clock()->now();
        auto delta_time = (time_now - time_prev_).nanoseconds() * 10e-9;
        auto error_eRPM = (target_eRPM - current_eRPM) * delta_time;

        double control_kp = kp_ * error_eRPM;
        double control_ki = control_ki_prev_ + ki_ * error_eRPM;
        double control_kd = (error_eRPM_prev_ - error_eRPM) * kd_;

        control_ki_prev_ = control_ki;
        error_eRPM_prev_ = error_eRPM;
        time_prev_ = time_now;

        auto msg_speed = float64msg();
        msg_speed.data = control_kp + control_ki + control_kd;
        motor_pub_->publish(msg_speed);
    }
};
} // namespace arc


int main(int argc, char **argv) {
    using namespace arc;

    // printf("Starting motor controller node");
    rclcpp::init(argc, argv);

    if (argc < 2) {
        std::exit(1);
    }

    int motor_id = std::atoi(argv[1]);
    if (motor_id != 0 || motor_id != 1) {
        return 1;
    }

    // Taken from VESC Tool
    double kp = 0.004;
    double ki = 0.004;
    double kd = 0.0001;

    arc::MotorPosition motor_pos =
        motor_id == 0 ? arc::MotorPosition::Left : arc::MotorPosition::Right;

    auto node = std::make_shared<arc::MotorController>(kp, ki, kd, motor_pos);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}