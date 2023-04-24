#include "rclcpp/rclcpp.hpp"

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

#define DBG(msg)                                                               \
    std::cerr << green(__FILE__) << ":" << yellow(std::to_string(__LINE__))    \
              << " " << blue(__FUNCTION__) << "() " << magenta(msg)            \
              << std::endl;

namespace ros2 = rclcpp;
using String = std::string;
template <typename T> using Vec = std::vector<T>;
template <typename Return, typename... Args>
using Fn = std::function<Return(Args...)>;

// pid control struct
struct PID {
    double p;
    double i;
    double d;
};

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