#include <rclcpp/rclcpp.hpp>

#include <tf2/impl/utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>

#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "f1tenth_simulator/pose_2d.hpp"
#include "f1tenth_simulator/ackermann_kinematics.hpp"
#include "f1tenth_simulator/scan_simulator_2d.hpp"
#include "f1tenth_simulator/car_state.hpp"
#include "f1tenth_simulator/car_params.hpp"
#include "f1tenth_simulator/ks_kinematics.hpp"
#include "f1tenth_simulator/st_kinematics.hpp"
#include "f1tenth_simulator/precompute.hpp"

#include <algorithm>
#include <cmath>
#include <vector>

using namespace racecar_simulator;

class RacecarSimulator : public rclcpp::Node {
private:
    std::string map_frame_, base_frame_, scan_frame_, odom_frame_;

    std::vector<int> added_obs_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obs_sub_;
    int obstacle_size_;

    CarState state_;
    double previous_seconds_;
    double scan_distance_to_base_link_;
    double max_speed_, max_steering_angle_;
    double max_accel_, max_steering_vel_, max_decel_;
    double desired_speed_, desired_steer_ang_;
    double accel_, steer_angle_vel_;
    CarParams params_;
    double width_;

    ScanSimulator2D scan_simulator_;
    double map_free_threshold_;

    tf2_ros::TransformBroadcaster br_;
    tf2_ros::StaticTransformBroadcaster static_br_;
    rclcpp::TimerBase::SharedPtr update_pose_timer_;

    rclcpp::Subscription<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_sub_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    bool map_exists_ = false;

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_rviz_sub_;

    bool broadcast_transform_;
    bool pub_gt_pose_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr scan_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;

    nav_msgs::msg::OccupancyGrid original_map_;
    nav_msgs::msg::OccupancyGrid current_map_;

    int map_width_ = 0;
    int map_height_ = 0;
    double map_resolution_ = 0.0;
    double origin_x_ = 0.0;
    double origin_y_ = 0.0;

    double thresh_;
    double speed_clip_diff_;

    std::vector<double> cosines_;

    double scan_fov_;
    double scan_ang_incr_;

    std::vector<double> car_distances_;

    bool TTC_ = false;
    double ttc_threshold_;

    int buffer_length_;
    std::vector<double> steering_buffer_;

public:
    RacecarSimulator()
        : Node("racecar_simulator"),
          br_(this),
          static_br_(this) {
        state_ = CarState();
        state_.x = 0.0;
        state_.y = 0.0;
        state_.theta = 0.0;
        state_.velocity = 0.0;
        state_.steer_angle = 0.0;
        state_.angular_velocity = 0.0;
        state_.slip_angle = 0.0;
        state_.st_dyn = false;
        accel_ = 0.0;
        steer_angle_vel_ = 0.0;
        desired_speed_ = 0.0;
        desired_steer_ang_ = 0.0;
        previous_seconds_ = this->now().seconds();

        this->declare_parameter<std::string>("drive_topic", "/drive");
        this->declare_parameter<std::string>("map_topic", "/map");
        this->declare_parameter<std::string>("scan_topic", "/scan");
        this->declare_parameter<std::string>("pose_topic", "/pose");
        this->declare_parameter<std::string>("odom_topic", "/odom");
        this->declare_parameter<std::string>("pose_rviz_topic", "/initialpose");
        this->declare_parameter<std::string>("imu_topic", "/imu");
        this->declare_parameter<std::string>("ground_truth_pose_topic", "/gt_pose");

        this->declare_parameter<int>("buffer_length", 5);

        this->declare_parameter<std::string>("map_frame", "map");
        this->declare_parameter<std::string>("base_frame", "base_link");
        this->declare_parameter<std::string>("scan_frame", "laser");
        this->declare_parameter<std::string>("odom_frame", "odom");

        this->declare_parameter<double>("wheelbase", 0.3302);
        this->declare_parameter<double>("update_pose_rate", 0.001);
        this->declare_parameter<int>("scan_beams", 1080);
        this->declare_parameter<double>("scan_field_of_view", 6.2831853);
        this->declare_parameter<double>("scan_std_dev", 0.01);
        this->declare_parameter<double>("map_free_threshold", 0.8);
        this->declare_parameter<double>("scan_distance_to_base_link", 0.275);
        this->declare_parameter<double>("max_speed", 7.0);
        this->declare_parameter<double>("max_steering_angle", 0.4189);
        this->declare_parameter<double>("max_accel", 7.51);
        this->declare_parameter<double>("max_decel", 8.26);
        this->declare_parameter<double>("max_steering_vel", 3.2);
        this->declare_parameter<double>("friction_coeff", 0.523);
        this->declare_parameter<double>("height_cg", 0.074);
        this->declare_parameter<double>("l_cg2rear", 0.17145);
        this->declare_parameter<double>("l_cg2front", 0.15875);
        this->declare_parameter<double>("C_S_front", 4.718);
        this->declare_parameter<double>("C_S_rear", 5.4562);
        this->declare_parameter<double>("moment_inertia", 0.04712);
        this->declare_parameter<double>("mass", 3.47);
        this->declare_parameter<double>("width", 0.2032);
        this->declare_parameter<double>("speed_clip_diff", 0.0);
        this->declare_parameter<bool>("broadcast_transform", true);
        this->declare_parameter<bool>("publish_ground_truth_pose", true);
        this->declare_parameter<int>("obstacle_size", 2);
        this->declare_parameter<double>("coll_threshold", 0.0);
        this->declare_parameter<double>("ttc_threshold", 0.01);

        const auto drive_topic = this->get_parameter("drive_topic").as_string();
        const auto map_topic = this->get_parameter("map_topic").as_string();
        const auto scan_topic = this->get_parameter("scan_topic").as_string();
        const auto pose_topic = this->get_parameter("pose_topic").as_string();
        const auto odom_topic = this->get_parameter("odom_topic").as_string();
        const auto pose_rviz_topic = this->get_parameter("pose_rviz_topic").as_string();
        const auto imu_topic = this->get_parameter("imu_topic").as_string();
        const auto gt_pose_topic = this->get_parameter("ground_truth_pose_topic").as_string();

        buffer_length_ = this->get_parameter("buffer_length").as_int();
        map_frame_ = this->get_parameter("map_frame").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        scan_frame_ = this->get_parameter("scan_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();

        const int scan_beams = this->get_parameter("scan_beams").as_int();
        const double update_pose_rate = this->get_parameter("update_pose_rate").as_double();
        const double scan_std_dev = this->get_parameter("scan_std_dev").as_double();

        params_.wheelbase = this->get_parameter("wheelbase").as_double();
        scan_fov_ = this->get_parameter("scan_field_of_view").as_double();
        map_free_threshold_ = this->get_parameter("map_free_threshold").as_double();
        scan_distance_to_base_link_ = this->get_parameter("scan_distance_to_base_link").as_double();
        max_speed_ = this->get_parameter("max_speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        max_accel_ = this->get_parameter("max_accel").as_double();
        max_decel_ = this->get_parameter("max_decel").as_double();
        max_steering_vel_ = this->get_parameter("max_steering_vel").as_double();
        params_.friction_coeff = this->get_parameter("friction_coeff").as_double();
        params_.h_cg = this->get_parameter("height_cg").as_double();
        params_.l_r = this->get_parameter("l_cg2rear").as_double();
        params_.l_f = this->get_parameter("l_cg2front").as_double();
        params_.cs_f = this->get_parameter("C_S_front").as_double();
        params_.cs_r = this->get_parameter("C_S_rear").as_double();
        params_.I_z = this->get_parameter("moment_inertia").as_double();
        params_.mass = this->get_parameter("mass").as_double();
        width_ = this->get_parameter("width").as_double();
        speed_clip_diff_ = this->get_parameter("speed_clip_diff").as_double();
        broadcast_transform_ = this->get_parameter("broadcast_transform").as_bool();
        pub_gt_pose_ = this->get_parameter("publish_ground_truth_pose").as_bool();
        obstacle_size_ = this->get_parameter("obstacle_size").as_int();
        thresh_ = this->get_parameter("coll_threshold").as_double();
        ttc_threshold_ = this->get_parameter("ttc_threshold").as_double();

        scan_simulator_ = ScanSimulator2D(scan_beams, scan_fov_, scan_std_dev);

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        map_qos.reliable();
        map_qos.transient_local();

        scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(scan_topic, rclcpp::QoS(10));
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(odom_topic, rclcpp::QoS(10));
        imu_pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, rclcpp::QoS(10));
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>(map_topic, map_qos);
        pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(gt_pose_topic, rclcpp::QoS(10));

        update_pose_timer_ = this->create_wall_timer(
            std::chrono::duration<double>(update_pose_rate),
            std::bind(&RacecarSimulator::update_pose, this));

        drive_sub_ = this->create_subscription<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic,
            rclcpp::QoS(10),
            std::bind(&RacecarSimulator::drive_callback, this, std::placeholders::_1));
        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic,
            map_qos,
            std::bind(&RacecarSimulator::map_callback, this, std::placeholders::_1));
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            pose_topic,
            rclcpp::QoS(10),
            std::bind(&RacecarSimulator::pose_callback, this, std::placeholders::_1));
        pose_rviz_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            pose_rviz_topic,
            rclcpp::QoS(10),
            std::bind(&RacecarSimulator::pose_rviz_callback, this, std::placeholders::_1));
        obs_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/clicked_point",
            rclcpp::QoS(10),
            std::bind(&RacecarSimulator::obs_callback, this, std::placeholders::_1));

        scan_ang_incr_ = scan_simulator_.get_angle_increment();
        cosines_ = Precompute::get_cosines(scan_beams, -scan_fov_ / 2.0, scan_ang_incr_);
        car_distances_ = Precompute::get_car_distances(
            scan_beams,
            params_.wheelbase,
            width_,
            scan_distance_to_base_link_,
            -scan_fov_ / 2.0,
            scan_ang_incr_);

        steering_buffer_ = std::vector<double>(buffer_length_);

        // Publish laser link as a static transform — the sensor position is
        // fixed relative to base_link and never changes at runtime.
        pub_laser_link_transform();
        RCLCPP_INFO(this->get_logger(), "Simulator constructed.");
    }

private:
    void update_pose() {
        compute_accel(desired_speed_);

        double actual_ang = 0.0;
        if (static_cast<int>(steering_buffer_.size()) < buffer_length_) {
            steering_buffer_.push_back(desired_steer_ang_);
            actual_ang = 0.0;
        } else {
            steering_buffer_.insert(steering_buffer_.begin(), desired_steer_ang_);
            actual_ang = steering_buffer_.back();
            steering_buffer_.pop_back();
        }
        set_steer_angle_vel(compute_steer_vel(actual_ang));

        const rclcpp::Time timestamp = this->now();
        const double current_seconds = timestamp.seconds();
        state_ = STKinematics::update(
            state_,
            accel_,
            steer_angle_vel_,
            params_,
            current_seconds - previous_seconds_);
        state_.velocity = std::min(std::max(state_.velocity, -max_speed_), max_speed_);
        state_.steer_angle = std::min(std::max(state_.steer_angle, -max_steering_angle_), max_steering_angle_);

        previous_seconds_ = current_seconds;

        pub_pose_transform(timestamp);
        pub_steer_ang_transform(timestamp);
        pub_odom(timestamp);
        pub_imu(timestamp);

        if (map_exists_) {
            Pose2D scan_pose;
            scan_pose.x = state_.x + scan_distance_to_base_link_ * std::cos(state_.theta);
            scan_pose.y = state_.y + scan_distance_to_base_link_ * std::sin(state_.theta);
            scan_pose.theta = state_.theta;

            const std::vector<double> scan = scan_simulator_.scan(scan_pose);
            std::vector<float> scan_(scan.size());
            for (size_t i = 0; i < scan.size(); i++) {
                scan_[i] = static_cast<float>(scan[i]);
            }

            bool no_collision = true;
            if (state_.velocity != 0) {
                for (size_t i = 0; i < scan_.size() && i < cosines_.size() && i < car_distances_.size(); i++) {
                    const double proj_velocity = state_.velocity * cosines_[i];
                    if (std::abs(proj_velocity) < 1e-6) {
                        continue;
                    }
                    const double ttc = (scan_[i] - car_distances_[i]) / proj_velocity;
                    if ((ttc < ttc_threshold_) && (ttc >= 0.0)) {
                        if (!TTC_) {
                            first_ttc_actions();
                        }

                        no_collision = false;
                        TTC_ = true;

                        RCLCPP_INFO(this->get_logger(), "Collision detected");
                    }
                }
            }

            if (no_collision) {
                TTC_ = false;
            }

            sensor_msgs::msg::LaserScan scan_msg;
            scan_msg.header.stamp = timestamp;
            scan_msg.header.frame_id = scan_frame_;
            scan_msg.angle_min = -scan_simulator_.get_field_of_view() / 2.;
            scan_msg.angle_max = scan_simulator_.get_field_of_view() / 2.;
            scan_msg.angle_increment = scan_simulator_.get_angle_increment();
            scan_msg.range_max = 100;
            scan_msg.ranges = scan_;
            scan_msg.intensities = scan_;

            scan_pub_->publish(scan_msg);
        }
    }

    std::vector<int> ind_2_rc(int ind) const {
        std::vector<int> rc;
        const int row = static_cast<int>(std::floor(static_cast<double>(ind) / map_width_));
        const int col = ind % map_width_ - 1;
        rc.push_back(row);
        rc.push_back(col);
        return rc;
    }

    int rc_2_ind(int r, int c) const {
        return r * map_width_ + c;
    }

    std::vector<int> coord_2_cell_rc(double x, double y) const {
        std::vector<int> rc;
        rc.push_back(static_cast<int>((y - origin_y_) / map_resolution_));
        rc.push_back(static_cast<int>((x - origin_x_) / map_resolution_));
        return rc;
    }

    void first_ttc_actions() {
        state_.velocity = 0.0;
        state_.angular_velocity = 0.0;
        state_.slip_angle = 0.0;
        state_.steer_angle = 0.0;
        steer_angle_vel_ = 0.0;
        accel_ = 0.0;
        desired_speed_ = 0.0;
        desired_steer_ang_ = 0.0;
    }

    void set_accel(double accel) {
        accel_ = std::min(std::max(accel, -max_accel_), max_accel_);
    }

    void set_steer_angle_vel(double steer_angle_vel) {
        steer_angle_vel_ = std::min(std::max(steer_angle_vel, -max_steering_vel_), max_steering_vel_);
    }

    void add_obs(int ind) {
        if (!map_exists_) {
            return;
        }
        const auto rc = ind_2_rc(ind);
        for (int i = -obstacle_size_; i < obstacle_size_; i++) {
            for (int j = -obstacle_size_; j < obstacle_size_; j++) {
                const int current_r = rc[0] + i;
                const int current_c = rc[1] + j;
                if (current_r < 0 || current_c < 0 || current_r >= map_height_ || current_c >= map_width_) {
                    continue;
                }
                const int current_ind = rc_2_ind(current_r, current_c);
                if (current_ind >= 0 && current_ind < static_cast<int>(current_map_.data.size())) {
                    current_map_.data[current_ind] = 100;
                }
            }
        }
        map_pub_->publish(current_map_);
    }

    double compute_steer_vel(double desired_angle) const {
        const double dif = (desired_angle - state_.steer_angle);
        if (std::abs(dif) > .0001) {
            return dif / std::abs(dif) * max_steering_vel_;
        }
        return 0.0;
    }

    void compute_accel(double desired_velocity) {
        const double dif = (desired_velocity - state_.velocity);

        if (state_.velocity > 0) {
            if (dif > 0) {
                const double kp = 2.0 * max_accel_ / max_speed_;
                set_accel(kp * dif);
            } else {
                accel_ = -max_decel_;
            }
        } else if (state_.velocity < 0) {
            if (dif > 0) {
                accel_ = max_decel_;
            } else {
                const double kp = 2.0 * max_accel_ / max_speed_;
                set_accel(kp * dif);
            }
        } else {
            const double kp = 2.0 * max_accel_ / max_speed_;
            set_accel(kp * dif);
        }
    }

    void obs_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        if (!map_exists_) {
            return;
        }
        const double x = msg->point.x;
        const double y = msg->point.y;
        const auto rc = coord_2_cell_rc(x, y);
        const int ind = rc_2_ind(rc[0], rc[1]);
        added_obs_.push_back(ind);
        add_obs(ind);
    }

    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        state_.x = msg->pose.position.x;
        state_.y = msg->pose.position.y;
        const geometry_msgs::msg::Quaternion q = msg->pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        state_.theta = tf2::impl::getYaw(quat);
    }

    void pose_rviz_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg) {
        auto temp_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        temp_pose->header = msg->header;
        temp_pose->pose = msg->pose.pose;
        pose_callback(temp_pose);
    }

    void drive_callback(const ackermann_msgs::msg::AckermannDriveStamped::SharedPtr msg) {
        desired_speed_ = msg->drive.speed;
        desired_steer_ang_ = msg->drive.steering_angle;
    }

    void map_callback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
        const size_t height = msg->info.height;
        const size_t width = msg->info.width;
        const double resolution = msg->info.resolution;

        Pose2D origin;
        origin.x = msg->info.origin.position.x;
        origin.y = msg->info.origin.position.y;
        const geometry_msgs::msg::Quaternion q = msg->info.origin.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        origin.theta = tf2::impl::getYaw(quat);

        std::vector<double> map(msg->data.size());
        for (size_t i = 0; i < height * width; i++) {
            if (msg->data[i] > 100 || msg->data[i] < 0) {
                map[i] = 0.5;
            } else {
                map[i] = msg->data[i] / 100.;
            }
        }

        scan_simulator_.set_map(map, height, width, resolution, origin, map_free_threshold_);
        map_exists_ = true;

        current_map_ = *msg;
        if (original_map_.data.empty()) {
            original_map_ = *msg;
        }

        map_width_ = static_cast<int>(msg->info.width);
        map_height_ = static_cast<int>(msg->info.height);
        origin_x_ = msg->info.origin.position.x;
        origin_y_ = msg->info.origin.position.y;
        map_resolution_ = msg->info.resolution;
    }

    void pub_pose_transform(const rclcpp::Time & timestamp) {
        geometry_msgs::msg::Transform t;
        t.translation.x = state_.x;
        t.translation.y = state_.y;
        tf2::Quaternion quat;
        quat.setEuler(0., 0., state_.theta);
        t.rotation.x = quat.x();
        t.rotation.y = quat.y();
        t.rotation.z = quat.z();
        t.rotation.w = quat.w();

        geometry_msgs::msg::PoseStamped ps;
        ps.header.frame_id = map_frame_;
        ps.header.stamp = timestamp;
        ps.pose.position.x = state_.x;
        ps.pose.position.y = state_.y;
        ps.pose.orientation.x = quat.x();
        ps.pose.orientation.y = quat.y();
        ps.pose.orientation.z = quat.z();
        ps.pose.orientation.w = quat.w();

        // Publish odom→base_link so SLAM can compute map→odom independently.
        // The ground truth pose is still published on /gt_pose in map frame.
        geometry_msgs::msg::TransformStamped ts;
        ts.transform = t;
        ts.header.stamp = timestamp;
        ts.header.frame_id = odom_frame_;
        ts.child_frame_id = base_frame_;

        if (broadcast_transform_) {
            br_.sendTransform(ts);
        }
        if (pub_gt_pose_) {
            pose_pub_->publish(ps);
        }
    }

    void pub_steer_ang_transform(const rclcpp::Time & timestamp) {
        tf2::Quaternion quat_wheel;
        quat_wheel.setEuler(0., 0., state_.steer_angle);

        geometry_msgs::msg::TransformStamped ts_wheel;
        ts_wheel.transform.rotation.x = quat_wheel.x();
        ts_wheel.transform.rotation.y = quat_wheel.y();
        ts_wheel.transform.rotation.z = quat_wheel.z();
        ts_wheel.transform.rotation.w = quat_wheel.w();
        ts_wheel.header.stamp = timestamp;
        ts_wheel.header.frame_id = "front_left_hinge";
        ts_wheel.child_frame_id = "front_left_wheel";
        br_.sendTransform(ts_wheel);

        ts_wheel.header.frame_id = "front_right_hinge";
        ts_wheel.child_frame_id = "front_right_wheel";
        br_.sendTransform(ts_wheel);
    }

    void pub_laser_link_transform() {
        geometry_msgs::msg::TransformStamped scan_ts;
        scan_ts.transform.translation.x = scan_distance_to_base_link_;
        scan_ts.transform.rotation.w = 1;
        scan_ts.header.stamp = rclcpp::Time(0);
        scan_ts.header.frame_id = base_frame_;
        scan_ts.child_frame_id = scan_frame_;
        static_br_.sendTransform(scan_ts);
    }

    void pub_odom(const rclcpp::Time & timestamp) {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = timestamp;
        odom.header.frame_id = odom_frame_;
        odom.child_frame_id = base_frame_;
        odom.pose.pose.position.x = state_.x;
        odom.pose.pose.position.y = state_.y;
        tf2::Quaternion quat;
        quat.setEuler(0., 0., state_.theta);
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();
        odom.twist.twist.linear.x = state_.velocity;
        odom.twist.twist.angular.z = state_.angular_velocity;
        odom_pub_->publish(odom);
    }

    void pub_imu(const rclcpp::Time & timestamp) {
        sensor_msgs::msg::Imu imu;
        imu.header.stamp = timestamp;
        imu.header.frame_id = map_frame_;
        imu_pub_->publish(imu);
    }
};

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RacecarSimulator>());
    rclcpp::shutdown();
    return 0;
}