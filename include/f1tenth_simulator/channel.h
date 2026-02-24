#ifndef CHANNEL_H
#define CHANNEL_H

#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>

class Mux;

class Channel {
private:
    // Publish drive data to simulator/car
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub;

    // Listen to drive data from a specific topic
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr channel_sub;

    // Mux index for this channel
    int mux_idx;

    // Pointer to mux object (to access mux controller and nodeHandle)
    Mux* mp_mux;


public:
    Channel();

    Channel(std::string channel_name, std::string drive_topic, int mux_idx_, Mux* mux);

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg);
};


#endif // CHANNEL_H