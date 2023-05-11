#pragma once

// #include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <iostream>
#include <iterator>
#include <sstream>
#include <string>
#include <type_traits>
#include <vector>

using namespace std::placeholders;

template <typename T> using Publisher = rclcpp::Publisher<T>;
template <typename T> using Subscriber = rclcpp::Subscription<T>;

auto green(std::string s) { return "\033[1;32m" + s + "\033[0m"; }
auto red(std::string s) { return "\033[1;31m" + s + "\033[0m"; }
auto yellow(std::string s) { return "\033[1;33m" + s + "\033[0m"; }
auto blue(std::string s) { return "\033[1;34m" + s + "\033[0m"; }
auto cyan(std::string s) { return "\033[1;36m" + s + "\033[0m"; }
auto magenta(std::string s) { return "\033[1;35m" + s + "\033[0m"; }
auto grey(std::string s) { return "\033[1;30m" + s + "\033[0m"; }

#define DBG(msg)                                                                                   \
    std::cerr << green(__FILE__) << ":" << yellow(std::to_string(__LINE__)) << " "                 \
              << blue(__FUNCTION__) << "() " << magenta(msg) << std::endl;

namespace ros2 = rclcpp;
using String = std::string;
template <typename T> using Vec = std::vector<T>;
template <typename Return, typename... Args> using Fn = std::function<Return(Args...)>;

// pid control struct
struct PID {
    double p;
    double i;
    double d;
};

template <typename Iterator>
std::string join(Iterator begin, Iterator end, const std::string &delimiter) {
    std::ostringstream oss;
    if (begin != end) {
        oss << *begin;
        ++begin;
    }

    while (begin != end) {
        oss << delimiter << *begin;
        ++begin;
    }

    return oss.str();
}

auto join(const std::vector<std::string> &strings, const std::string &delimiter) {
    return join(std::begin(strings), std::end(strings), delimiter);
}

template <typename T>
auto declare_and_get_parameter(ros2::Node &node, const std::string &name,
                               const std::string &description = "") -> T {
    const std::vector<std::string> supported_types = {"int", "double", "string", "bool"};
    const std::string error_msg =
        "Unsupported type. Supported types are: " +
        join(std::begin(supported_types), std::end(supported_types), ", ");

    throw std::runtime_error(error_msg.c_str());
}

template <>
auto declare_and_get_parameter<int>(ros2::Node &node, const std::string &name,
                                    const std::string &description) -> int {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = description;
    const int default_value = 0;
    node.declare_parameter(name, default_value, param_desc);
    int param = node.get_parameter(name).as_int();
    std::fprintf(stderr, "%s: %d\n", magenta(name).c_str(), param);
    return param;
}

template <>
auto declare_and_get_parameter<double>(ros2::Node &node, const std::string &name,
                                       const std::string &description) -> double {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = description;
    const double default_value = 0.0;
    node.declare_parameter(name, default_value, param_desc);
    double param = node.get_parameter(name).as_double();
    std::fprintf(stderr, "%s: %f\n", magenta(name).c_str(), param);
    return param;
}

template <>
auto declare_and_get_parameter<std::string>(ros2::Node &node, const std::string &name,
                                            const std::string &description) -> std::string {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = description;
    const std::string default_value = "";
    node.declare_parameter(name, default_value, param_desc);
    std::string param = node.get_parameter(name).as_string();
    std::fprintf(stderr, "%s: %s\n", magenta(name).c_str(), param.c_str());
    return param;
}

template <>
auto declare_and_get_parameter<bool>(ros2::Node &node, const std::string &name,
                                     const std::string &description) -> bool {
    auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    param_desc.description = description;
    const bool default_value = false;
    node.declare_parameter(name, default_value, param_desc);
    bool param = node.get_parameter(name).as_bool();
    std::fprintf(stderr, "%s: %s\n", magenta(name).c_str(), param ? "true" : "false");
    return param;
}

// template <typename T>
// auto get_parameter<T>(ros2::Node &node, const std::string &name,
//                       const std::string &description = "") -> T {
//     auto param_desc = rcl_interfaces::msg::ParameterDescriptor{};
//     param_desc.description = description;

//     node.declare_parameter(name, param_desc);
//     std::fprintf(stderr, "%s: %s\n", magenta("motor_id").c_str(),
//                  motor_id.c_str());

//     // Depending on the type of T call the appropriate get_parameter method
//     // e.g. T = std::string -> get_parameter(name).as_string()
//     //      T = int         -> get_parameter(name).as_int()

//     constexpr if (std::is_same<T, std::string>::value) {
//         auto param = node.get_parameter(name).as_string();
//         std::fprintf(stderr, "%s: %s\n", magenta(name).c_str(),
//         param.c_str()); return param;
//     }
//     else if (std::is_same<T, int>::value) {
//         auto param = node.get_parameter(name).as_int();
//         std::fprintf(stderr, "%s: %d\n", magenta(name).c_str(), param);
//         return param;
//     }
//     else if (std::is_same<T, double>::value) {
//         auto param = node.get_parameter(name).as_double();
//         std::fprintf(stderr, "%s: %f\n", magenta(name).c_str(), param);
//         return param;
//     }
//     else if (std::is_same<T, bool>::value) {
//         auto param = node.get_parameter(name).as_bool();
//         std::fprintf(stderr, "%s: %s\n", magenta(name).c_str(),
//                      param ? "true" : "false");
//         return param;
//     }
//     else {
//         throw std::runtime_error("Unsupported parameter type");
//     }
// }

// A utility function that takes a variadic number of strings
// and concatenates them into a single string formatted as a ros2 topic path
template <typename... Args> auto format_topic_path(Args... args) -> String {
    const Vec<String> topic_path = {args...};
    if (!(topic_path.size() > 0)) {
        DBG("topic_path.size() <= 0");
        ros2::shutdown();
    }
    String topic_path_str = "";
    if (topic_path.size() == 1) {
        topic_path_str = topic_path[0];
    } else {
        for (std::size_t i = 0; i < topic_path.size() - 1; i++) {
            topic_path_str += topic_path[i] + "/";
        }
        topic_path_str += topic_path.back();
    }
    // RCLCPP_DEBUG(ros2::get_logger(), "topic: " + yellow(topic_path_str));
    return topic_path_str;
}

struct RobotParameters {
    double wheel_diameter;
    double wheel_base;
    int motor_pulley_teeth;
    int wheel_pulley_teeth;
    double gear_ratio;
    int motor_pole_pairs;

    RobotParameters() = default;

    RobotParameters(double wheel_diameter, double wheel_base, int motor_pulley_teeth,
                    int wheel_pulley_teeth, int motor_pole_pairs)
        : wheel_diameter(wheel_diameter), wheel_base(wheel_base),
          motor_pulley_teeth(motor_pulley_teeth), wheel_pulley_teeth(wheel_pulley_teeth),
          motor_pole_pairs(motor_pole_pairs) {
        gear_ratio = (double)wheel_pulley_teeth / (double)motor_pulley_teeth;
    }

    RobotParameters(ros2::Node &node) {
        // get ros parameters
        String prefix = "robot.";
        wheel_diameter = declare_and_get_parameter<double>(node, prefix + "wheel_diameter",
                                                           "Wheel radius in meters");
        wheel_base =
            declare_and_get_parameter<double>(node, prefix + "wheel_base", "Wheel base in meters");
        motor_pulley_teeth = declare_and_get_parameter<int>(node, prefix + "motor_pulley_teeth",
                                                            "Motor pulley teeth");
        wheel_pulley_teeth = declare_and_get_parameter<int>(node, prefix + "wheel_pulley_teeth",
                                                            "Wheel pulley teeth");
        motor_pole_pairs =
            declare_and_get_parameter<int>(node, prefix + "motor_pole_pairs", "Motor pole pairs");
        gear_ratio = (double)wheel_pulley_teeth / (double)motor_pulley_teeth;
    }
};
