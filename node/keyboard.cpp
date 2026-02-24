#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>

#include <termios.h>

#include <signal.h>

static volatile sig_atomic_t keep_running = 1;

void sigHandler(int) {
    keep_running = 0;
}

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("keyboard");

    node->declare_parameter<std::string>("keyboard_topic", "/key");
    const auto keyboard_topic = node->get_parameter("keyboard_topic").as_string();

    auto key_pub = node->create_publisher<std_msgs::msg::String>(keyboard_topic, rclcpp::QoS(10));

    static struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON);
    tcsetattr(STDIN_FILENO, 0, &newt);

    struct sigaction act;
    act.sa_handler = sigHandler;
    sigaction(SIGINT, &act, nullptr);

    std_msgs::msg::String msg;
    while (rclcpp::ok() && keep_running) {
        const int c = getchar();
        if (c == EOF) {
            continue;
        }
        msg.data = std::string(1, static_cast<char>(c));
        key_pub->publish(msg);
    }

    tcsetattr(STDIN_FILENO, 0, &oldt);

    rclcpp::shutdown();
    return 0;
}