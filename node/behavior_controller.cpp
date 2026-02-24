#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/int32_multi_array.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/string.hpp>

#include <fstream>
#include <filesystem>

#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/precompute.hpp"

using namespace racecar_simulator;

class BehaviorController : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr key_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr brake_bool_sub_;

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr mux_pub_;

    int joy_mux_idx_;
    int key_mux_idx_;
    int random_walker_mux_idx_;
    int nav_mux_idx_;
    int brake_mux_idx_;

    std::vector<bool> mux_controller_;
    int mux_size_;

    int joy_button_idx_;
    int key_button_idx_;
    int random_walk_button_idx_;
    int brake_button_idx_;
    int nav_button_idx_;

    std::string joy_key_char_;
    std::string keyboard_key_char_;
    std::string brake_key_char_;
    std::string random_walk_key_char_;
    std::string nav_key_char_;

    bool safety_on_;
    racecar_simulator::CarState state_;

    std::vector<double> car_distances_;
    std::vector<double> cosines_;

    double ttc_threshold_;
    bool in_collision_ = false;

    std::ofstream collision_file_;
    double beginning_seconds_;
    int collision_count_ = 0;

public:
    BehaviorController() : Node("behavior_controller") {
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("joy_topic", "/joy");
        this->declare_parameter<std::string>("mux_topic", "/mux");
        this->declare_parameter<std::string>("keyboard_topic", "/key");
        this->declare_parameter<std::string>("brake_bool_topic", "/brake_bool");

        this->declare_parameter<int>("joy_mux_idx", 0);
        this->declare_parameter<int>("key_mux_idx", 1);
        this->declare_parameter<int>("random_walker_mux_idx", 2);
        this->declare_parameter<int>("brake_mux_idx", 3);
        this->declare_parameter<int>("nav_mux_idx", 4);

        this->declare_parameter<int>("joy_button_idx", 4);
        this->declare_parameter<int>("key_button_idx", 6);
        this->declare_parameter<int>("random_walk_button_idx", 1);
        this->declare_parameter<int>("brake_button_idx", 0);
        this->declare_parameter<int>("nav_button_idx", 5);

        this->declare_parameter<std::string>("joy_key_char", "j");
        this->declare_parameter<std::string>("keyboard_key_char", "k");
        this->declare_parameter<std::string>("random_walk_key_char", "r");
        this->declare_parameter<std::string>("brake_key_char", "b");
        this->declare_parameter<std::string>("nav_key_char", "n");

        this->declare_parameter<int>("mux_size", 5);
        this->declare_parameter<double>("ttc_threshold", 0.01);
        this->declare_parameter<int>("scan_beams", 1080);
        this->declare_parameter<double>("scan_distance_to_base_link", 0.275);
        this->declare_parameter<double>("width", 0.2032);
        this->declare_parameter<double>("wheelbase", 0.3302);
        this->declare_parameter<double>("scan_field_of_view", 6.2831853);
        this->declare_parameter<std::string>("collision_file", "collision_file");

        const auto scan_topic = this->get_parameter("scan_topic").as_string();
        const auto odom_topic = this->get_parameter("odom_topic").as_string();
        const auto imu_topic = this->get_parameter("imu_topic").as_string();
        const auto joy_topic = this->get_parameter("joy_topic").as_string();
        const auto mux_topic = this->get_parameter("mux_topic").as_string();
        const auto keyboard_topic = this->get_parameter("keyboard_topic").as_string();
        const auto brake_bool_topic = this->get_parameter("brake_bool_topic").as_string();

        joy_mux_idx_ = this->get_parameter("joy_mux_idx").as_int();
        key_mux_idx_ = this->get_parameter("key_mux_idx").as_int();
        random_walker_mux_idx_ = this->get_parameter("random_walker_mux_idx").as_int();
        brake_mux_idx_ = this->get_parameter("brake_mux_idx").as_int();
        nav_mux_idx_ = this->get_parameter("nav_mux_idx").as_int();

        joy_button_idx_ = this->get_parameter("joy_button_idx").as_int();
        key_button_idx_ = this->get_parameter("key_button_idx").as_int();
        random_walk_button_idx_ = this->get_parameter("random_walk_button_idx").as_int();
        brake_button_idx_ = this->get_parameter("brake_button_idx").as_int();
        nav_button_idx_ = this->get_parameter("nav_button_idx").as_int();

        joy_key_char_ = this->get_parameter("joy_key_char").as_string();
        keyboard_key_char_ = this->get_parameter("keyboard_key_char").as_string();
        random_walk_key_char_ = this->get_parameter("random_walk_key_char").as_string();
        brake_key_char_ = this->get_parameter("brake_key_char").as_string();
        nav_key_char_ = this->get_parameter("nav_key_char").as_string();

        mux_size_ = this->get_parameter("mux_size").as_int();
        mux_controller_.assign(mux_size_, false);

        safety_on_ = false;

        state_ = CarState();
        state_.x = 0.0;
        state_.y = 0.0;
        state_.theta = 0.0;
        state_.velocity = 0.0;
        state_.steer_angle = 0.0;
        state_.angular_velocity = 0.0;
        state_.slip_angle = 0.0;
        state_.st_dyn = false;

        const int scan_beams = this->get_parameter("scan_beams").as_int();
        const double scan_distance_to_base_link = this->get_parameter("scan_distance_to_base_link").as_double();
        const double width = this->get_parameter("width").as_double();
        const double wheelbase = this->get_parameter("wheelbase").as_double();
        const double scan_fov = this->get_parameter("scan_field_of_view").as_double();
        const double scan_ang_incr = scan_fov / scan_beams;
        ttc_threshold_ = this->get_parameter("ttc_threshold").as_double();

        cosines_ = Precompute::get_cosines(scan_beams, -scan_fov / 2.0, scan_ang_incr);
        car_distances_ = Precompute::get_car_distances(scan_beams, wheelbase, width, scan_distance_to_base_link, -scan_fov / 2.0, scan_ang_incr);

        const auto filename = this->get_parameter("collision_file").as_string();
        const auto log_dir = std::filesystem::current_path() / "logs";
        std::filesystem::create_directories(log_dir);
        collision_file_.open((log_dir / (filename + ".txt")).string());
        beginning_seconds_ = this->now().seconds();

        mux_pub_ = this->create_publisher<std_msgs::msg::Int32MultiArray>(mux_topic, rclcpp::QoS(10));

        laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            scan_topic,
            rclcpp::QoS(10),
            std::bind(&BehaviorController::laser_callback, this, std::placeholders::_1));
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            joy_topic,
            rclcpp::QoS(10),
            std::bind(&BehaviorController::joy_callback, this, std::placeholders::_1));
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic,
            rclcpp::QoS(10),
            std::bind(&BehaviorController::imu_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic,
            rclcpp::QoS(10),
            std::bind(&BehaviorController::odom_callback, this, std::placeholders::_1));
        key_sub_ = this->create_subscription<std_msgs::msg::String>(
            keyboard_topic,
            rclcpp::QoS(10),
            std::bind(&BehaviorController::key_callback, this, std::placeholders::_1));
        brake_bool_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            brake_bool_topic,
            rclcpp::QoS(10),
            std::bind(&BehaviorController::brake_callback, this, std::placeholders::_1));
    }

private:
    void publish_mux() {
        std_msgs::msg::Int32MultiArray mux_msg;
        mux_msg.data.reserve(mux_size_);
        for (int i = 0; i < mux_size_; i++) {
            mux_msg.data.push_back(static_cast<int32_t>(mux_controller_[i]));
        }
        mux_pub_->publish(mux_msg);
    }

    void change_controller(int controller_idx) {
        for (int i = 0; i < mux_size_; i++) {
            mux_controller_[i] = false;
        }
        if (controller_idx >= 0 && controller_idx < mux_size_) {
            mux_controller_[controller_idx] = true;
        }
        publish_mux();
    }

    void collision_checker(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (state_.velocity == 0.0) {
            return;
        }
        for (size_t i = 0; i < msg->ranges.size() && i < cosines_.size() && i < car_distances_.size(); i++) {
            const double angle = msg->angle_min + i * msg->angle_increment;
            const double proj_velocity = state_.velocity * cosines_[i];
            if (std::abs(proj_velocity) < 1e-6) {
                continue;
            }
            const double ttc = (msg->ranges[i] - car_distances_[i]) / proj_velocity;

            if ((ttc < ttc_threshold_) && (ttc >= 0.0)) {
                collision_helper();
                in_collision_ = true;

                collision_count_++;
                collision_file_ << "Collision #" << collision_count_ << " detected:\n";
                collision_file_ << "TTC: " << ttc << " seconds\n";
                collision_file_ << "Angle to obstacle: " << angle << " radians\n";
                collision_file_ << "Time since start of sim: " << (this->now().seconds() - beginning_seconds_) << " seconds\n\n";
                return;
            }
        }
        in_collision_ = false;
    }

    void collision_helper() {
        safety_on_ = false;
        for (int i = 0; i < mux_size_; i++) {
            mux_controller_[i] = false;
        }
        publish_mux();
    }

    void toggle_mux(int mux_idx, const std::string & driver_name) {
        if (mux_idx < 0 || mux_idx >= mux_size_) {
            return;
        }
        if (mux_controller_[mux_idx]) {
            RCLCPP_INFO(this->get_logger(), "%s turned off", driver_name.c_str());
            mux_controller_[mux_idx] = false;
            publish_mux();
        } else {
            RCLCPP_INFO(this->get_logger(), "%s turned on", driver_name.c_str());
            change_controller(mux_idx);
        }
    }

    void toggle_brake_mux() {
        RCLCPP_INFO(this->get_logger(), "Emergency brake engaged");
        for (int i = 0; i < mux_size_; i++) {
            mux_controller_[i] = false;
        }
        if (brake_mux_idx_ >= 0 && brake_mux_idx_ < mux_size_) {
            mux_controller_[brake_mux_idx_] = true;
        }
        publish_mux();
    }

    void brake_callback(const std_msgs::msg::Bool::SharedPtr msg) {
        if (msg->data && safety_on_) {
            toggle_brake_mux();
        } else if (!msg->data && brake_mux_idx_ >= 0 && brake_mux_idx_ < mux_size_ && mux_controller_[brake_mux_idx_]) {
            mux_controller_[brake_mux_idx_] = false;
        }
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        auto pressed = [msg](int idx) -> bool {
            return idx >= 0 && idx < static_cast<int>(msg->buttons.size()) && msg->buttons[idx];
        };

        if (pressed(joy_button_idx_)) {
            toggle_mux(joy_mux_idx_, "Joystick");
        }
        if (pressed(key_button_idx_)) {
            toggle_mux(key_mux_idx_, "Keyboard");
        } else if (pressed(brake_button_idx_)) {
            if (safety_on_) {
                RCLCPP_INFO(this->get_logger(), "Emergency brake turned off");
                safety_on_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Emergency brake turned on");
                safety_on_ = true;
            }
        } else if (pressed(random_walk_button_idx_)) {
            toggle_mux(random_walker_mux_idx_, "Random Walker");
        } else if (pressed(nav_button_idx_)) {
            toggle_mux(nav_mux_idx_, "Navigation");
        }
    }

    void key_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == joy_key_char_) {
            toggle_mux(joy_mux_idx_, "Joystick");
        } else if (msg->data == keyboard_key_char_) {
            toggle_mux(key_mux_idx_, "Keyboard");
        } else if (msg->data == brake_key_char_) {
            if (safety_on_) {
                RCLCPP_INFO(this->get_logger(), "Emergency brake turned off");
                safety_on_ = false;
            } else {
                RCLCPP_INFO(this->get_logger(), "Emergency brake turned on");
                safety_on_ = true;
            }
        } else if (msg->data == random_walk_key_char_) {
            toggle_mux(random_walker_mux_idx_, "Random Walker");
        } else if (msg->data == nav_key_char_) {
            toggle_mux(nav_mux_idx_, "Navigation");
        }
    }

    void laser_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        collision_checker(msg);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        state_.velocity = msg->twist.twist.linear.x;
        state_.angular_velocity = msg->twist.twist.angular.z;
        state_.x = msg->pose.pose.position.x;
        state_.y = msg->pose.pose.position.y;
    }

    void imu_callback(const sensor_msgs::msg::Imu::SharedPtr) {
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BehaviorController>());
    rclcpp::shutdown();
    return 0;
}