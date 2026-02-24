#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <cstdlib>

class RandomWalker : public rclcpp::Node {
private:
    double max_speed_;
    double max_steering_angle_;

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    double prev_angle_ = 0.0;

public:
    RandomWalker() : Node("random_walker") {
        this->declare_parameter<std::string>("rand_drive_topic", "/rand_drive");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<double>("max_speed", 7.0);
        this->declare_parameter<double>("max_steering_angle", 0.4189);

        const auto drive_topic = this->get_parameter("rand_drive_topic").as_string();
        const auto odom_topic = this->get_parameter("odom_topic").as_string();
        max_speed_ = this->get_parameter("max_speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, rclcpp::QoS(10));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            rclcpp::QoS(10),
            std::bind(&RandomWalker::odom_callback, this, std::placeholders::_1));
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr) {
        ackermann_msgs::msg::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::msg::AckermannDrive drive_msg;

        drive_st_msg.header.stamp = this->now();
        drive_msg.speed = max_speed_ / 2.0;

        double random = static_cast<double>(rand()) / RAND_MAX;
        const double range = max_steering_angle_ / 2.0;
        double rand_ang = range * random - range / 2.0;

        random = static_cast<double>(rand()) / RAND_MAX;
        if ((random > .8) && (prev_angle_ != 0.0) && (rand_ang != 0.0)) {
            const double sign_rand = rand_ang / std::abs(rand_ang);
            const double sign_prev = prev_angle_ / std::abs(prev_angle_);
            rand_ang *= sign_rand * sign_prev;
        }

        drive_msg.steering_angle = std::min(std::max(prev_angle_ + rand_ang, -max_steering_angle_), max_steering_angle_);
        prev_angle_ = drive_msg.steering_angle;

        drive_st_msg.drive = drive_msg;
        drive_pub_->publish(drive_st_msg);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RandomWalker>());
    rclcpp::shutdown();
    return 0;
}