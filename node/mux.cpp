#include <rclcpp/rclcpp.hpp>

#include <ackermann_msgs/msg/ackermann_drive.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <iostream>
#include <memory>

class Mux;

class Channel {
public:
    Channel(
        rclcpp::Node * node,
        const std::string & channel_name,
        const std::string & drive_topic,
        int mux_idx,
        std::vector<bool> * mux_controller)
        : mux_idx_(mux_idx), mux_controller_(mux_controller) {
        drive_pub_ = node->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, rclcpp::QoS(10));
        channel_sub_ = node->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            channel_name,
            rclcpp::QoS(10),
            std::bind(&Channel::drive_callback, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr channel_sub_;
    int mux_idx_;
    std::vector<bool> * mux_controller_;

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        if (mux_controller_ != nullptr && mux_idx_ >= 0 && mux_idx_ < static_cast<int>(mux_controller_->size()) && (*mux_controller_)[mux_idx_]) {
            drive_pub_->publish(*msg);
        }
    }
};

class Mux : public rclcpp::Node {
private:
    rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr mux_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;

    int joy_mux_idx_;
    int key_mux_idx_;

    std::vector<bool> mux_controller_;
    int mux_size_;

    std::vector<std::unique_ptr<Channel>> channels_;
    std::vector<bool> prev_mux_;

    int joy_speed_axis_, joy_angle_axis_;
    double max_speed_, max_steering_angle_;

    double prev_key_velocity_ = 0.0;
    double keyboard_speed_;
    double keyboard_steer_ang_;

public:
    Mux() : Node("mux_controller") {
        this->declare_parameter<std::string>("drive_topic", "/drive");
        this->declare_parameter<std::string>("mux_topic", "/mux");
        this->declare_parameter<std::string>("joy_topic", "/joy");
        this->declare_parameter<std::string>("keyboard_topic", "/key");
        this->declare_parameter<int>("joy_mux_idx", 0);
        this->declare_parameter<int>("key_mux_idx", 1);
        this->declare_parameter<int>("joy_speed_axis", 1);
        this->declare_parameter<int>("joy_angle_axis", 3);
        this->declare_parameter<double>("max_steering_angle", 0.4189);
        this->declare_parameter<double>("max_speed", 7.0);
        this->declare_parameter<double>("keyboard_speed", 1.8);
        this->declare_parameter<double>("keyboard_steer_ang", 0.3);
        this->declare_parameter<int>("mux_size", 5);

        const auto drive_topic = this->get_parameter("drive_topic").as_string();
        const auto mux_topic = this->get_parameter("mux_topic").as_string();
        const auto joy_topic = this->get_parameter("joy_topic").as_string();
        const auto key_topic = this->get_parameter("keyboard_topic").as_string();

        joy_mux_idx_ = this->get_parameter("joy_mux_idx").as_int();
        key_mux_idx_ = this->get_parameter("key_mux_idx").as_int();
        joy_speed_axis_ = this->get_parameter("joy_speed_axis").as_int();
        joy_angle_axis_ = this->get_parameter("joy_angle_axis").as_int();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        keyboard_speed_ = this->get_parameter("keyboard_speed").as_double();
        keyboard_steer_ang_ = this->get_parameter("keyboard_steer_ang").as_double();
        mux_size_ = this->get_parameter("mux_size").as_int();

        mux_controller_.assign(mux_size_, false);
        prev_mux_.assign(mux_size_, false);

        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic, rclcpp::QoS(10));

        mux_sub_ = this->create_subscription<std_msgs::msg::Int32MultiArray>(
            mux_topic,
            rclcpp::QoS(10),
            std::bind(&Mux::mux_callback, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic,
            rclcpp::QoS(10),
            std::bind(&Mux::joy_callback, this, std::placeholders::_1));
        key_sub_ = this->create_subscription<std_msgs::msg::String>(
            key_topic,
            rclcpp::QoS(10),
            std::bind(&Mux::key_callback, this, std::placeholders::_1));

        int random_walker_mux_idx;
        int brake_mux_idx;
        int nav_mux_idx;
        std::string rand_drive_topic;
        std::string brake_drive_topic;
        std::string nav_drive_topic;

        this->declare_parameter<std::string>("rand_drive_topic", "/rand_drive");
        this->declare_parameter<int>("random_walker_mux_idx", 2);
        this->declare_parameter<std::string>("brake_drive_topic", "/brake");
        this->declare_parameter<int>("brake_mux_idx", 3);
        this->declare_parameter<std::string>("nav_drive_topic", "/nav");
        this->declare_parameter<int>("nav_mux_idx", 4);

        rand_drive_topic = this->get_parameter("rand_drive_topic").as_string();
        random_walker_mux_idx = this->get_parameter("random_walker_mux_idx").as_int();
        brake_drive_topic = this->get_parameter("brake_drive_topic").as_string();
        brake_mux_idx = this->get_parameter("brake_mux_idx").as_int();
        nav_drive_topic = this->get_parameter("nav_drive_topic").as_string();
        nav_mux_idx = this->get_parameter("nav_mux_idx").as_int();

        add_channel(rand_drive_topic, drive_topic, random_walker_mux_idx);
        add_channel(brake_drive_topic, drive_topic, brake_mux_idx);
        add_channel(nav_drive_topic, drive_topic, nav_mux_idx);
    }

private:
    void add_channel(const std::string & channel_name, const std::string & drive_topic, int mux_idx) {
        channels_.push_back(std::make_unique<Channel>(this, channel_name, drive_topic, mux_idx, &mux_controller_));
    }

    void publish_to_drive(double desired_velocity, double desired_steer) {
        ackermann_msgs::msg::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::msg::AckermannDrive drive_msg;

        drive_st_msg.header.stamp = this->now();
        drive_msg.speed = desired_velocity;
        drive_msg.steering_angle = desired_steer;
        drive_st_msg.drive = drive_msg;

        drive_pub_->publish(drive_st_msg);
    }

    void mux_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
        for (int i = 0; i < mux_size_ && i < static_cast<int>(msg->data.size()); i++) {
            mux_controller_[i] = static_cast<bool>(msg->data[i]);
        }

        bool changed = false;
        bool anything_on = false;
        for (int i = 0; i < mux_size_; i++) {
            changed = changed || (mux_controller_[i] != prev_mux_[i]);
            anything_on = anything_on || mux_controller_[i];
        }

        if (changed) {
            std::cout << "MUX:" << std::endl;
            for (int i = 0; i < mux_size_; i++) {
                std::cout << mux_controller_[i] << std::endl;
                prev_mux_[i] = mux_controller_[i];
            }
            std::cout << std::endl;
        }

        if (!anything_on) {
            publish_to_drive(0.0, 0.0);
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        if (joy_mux_idx_ < 0 || joy_mux_idx_ >= static_cast<int>(mux_controller_.size()) || !mux_controller_[joy_mux_idx_]) {
            return;
        }
        if (joy_speed_axis_ >= static_cast<int>(msg->axes.size()) || joy_angle_axis_ >= static_cast<int>(msg->axes.size())) {
            return;
        }

        const double desired_velocity = max_speed_ * msg->axes[joy_speed_axis_];
        const double desired_steer = max_steering_angle_ * msg->axes[joy_angle_axis_];
        publish_to_drive(desired_velocity, desired_steer);
    }

    void key_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (key_mux_idx_ < 0 || key_mux_idx_ >= static_cast<int>(mux_controller_.size()) || !mux_controller_[key_mux_idx_]) {
            return;
        }

        double desired_velocity = 0.0;
        double desired_steer = 0.0;
        bool publish = true;

        if (msg->data == "w") {
            desired_velocity = keyboard_speed_;
        } else if (msg->data == "s") {
            desired_velocity = -keyboard_speed_;
        } else if (msg->data == "a") {
            desired_steer = keyboard_steer_ang_;
            desired_velocity = prev_key_velocity_;
        } else if (msg->data == "d") {
            desired_steer = -keyboard_steer_ang_;
            desired_velocity = prev_key_velocity_;
        } else if (msg->data == " ") {
        } else {
            publish = false;
        }

        if (publish) {
            publish_to_drive(desired_velocity, desired_steer);
            prev_key_velocity_ = desired_velocity;
        }
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Mux>());
    rclcpp::shutdown();
    return 0;
}