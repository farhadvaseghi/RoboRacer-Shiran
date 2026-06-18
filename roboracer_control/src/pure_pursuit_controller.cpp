#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <stdexcept>
#include <string>

using namespace std::chrono_literals;

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController() : Node("pure_pursuit_controller")
    {
        declare_parameters();
        read_parameters();

        // -------------------------------------------------
        // ROS interfaces
        // -------------------------------------------------

        // Publish final Ackermann command
        nav_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            drive_topic_, 10
        );

        // RViz debug markers
        debug_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/control_debug_markers", 10
        );

        // Publish local tracking error
        tracking_error_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/tracking_error", 10
        );

        // Current robot state from odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_,
            10,
            std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1)
        );

        opponent_odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            opponent_odom_topic_,
            10,
            std::bind(&PurePursuitController::opponent_odom_callback, this, std::placeholders::_1)
        );

        // Global path from planner
        auto path_qos = rclcpp::QoS(rclcpp::KeepLast(1));
        path_qos.reliable().transient_local();
        plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            path_topic_,
            path_qos,
            std::bind(&PurePursuitController::path_callback, this, std::placeholders::_1)
        );

        // Main control loop
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period_sec_),
            std::bind(&PurePursuitController::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(),
                    "PurePursuitController: odom=%s opponent_odom=%s path=%s drive=%s mpc_overtaking=%s",
                    odom_topic_.c_str(), opponent_odom_topic_.c_str(), path_topic_.c_str(),
                    drive_topic_.c_str(), enable_mpc_overtaking_ ? "enabled" : "disabled");
    }

private:
    // Simple 2D point used for internal path storage
    struct Point2D
    {
        double x;
        double y;
    };

    struct TrackingErrors
    {
        double cross_track_error = 0.0;
        double heading_error = 0.0;
        double proj_x = 0.0;
        double proj_y = 0.0;
        double path_yaw = 0.0;
    };

    struct MpcDecision
    {
        bool active = false;
        bool opponent_relevant = false;
        double target_x = 0.0;
        double target_y = 0.0;
        double target_speed = 0.0;
        double lateral_offset = 0.0;
        double cost = 0.0;
        double min_predicted_clearance = std::numeric_limits<double>::infinity();
    };

    // -------------------------------------------------
    // ROS interfaces
    // -------------------------------------------------
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr nav_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr tracking_error_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr opponent_odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // -------------------------------------------------
    // Current robot state from /odom
    // -------------------------------------------------
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    double velocity_ = 0.0;
    bool have_odom_ = false;

    double opponent_x_ = 0.0;
    double opponent_y_ = 0.0;
    double opponent_yaw_ = 0.0;
    double opponent_speed_ = 0.0;
    bool have_opponent_ = false;
    rclcpp::Time last_opponent_stamp_;

    // -------------------------------------------------
    // Current path state from /plan
    // -------------------------------------------------
    std::vector<Point2D> path_;
    bool have_path_ = false;
    int last_progress_idx_ = 0;
    bool goal_reached_ = false;

    // Forward / reverse mode
    bool reverse_mode_ = false;

    // -------------------------------------------------
    // Controller parameters
    // -------------------------------------------------
    double wheelbase_;
    double lookahead_distance_;
    double min_lookahead_distance_;
    double max_lookahead_distance_;
    double lookahead_time_;
    double speed_;
    double reverse_speed_;
    double max_steering_angle_;
    double goal_tolerance_;
    double min_speed_near_goal_;
    double sharp_turn_threshold_;
    double goal_slowdown_distance_;
    double turn_speed_reduction_factor_;
    double straight_speed_;
    double turn_speed_;
    double close_obstacle_speed_;
    double obstacle_slowdown_distance_;
    double obstacle_stop_distance_;
    double max_acceleration_;
    double max_deceleration_;
    double pid_kp_;
    double pid_ki_;
    double pid_kd_;
    double pid_integral_limit_;
    double pid_output_limit_;
    double max_speed_command_;
    double reverse_trigger_x_;
    double forward_trigger_x_;
    bool use_velocity_scaled_lookahead_;
    bool use_speed_pid_;
    bool enable_mpc_overtaking_;
    bool mpc_overtake_active_ = false;
    int mpc_horizon_steps_;
    double mpc_dt_;
    double mpc_safety_radius_;
    double mpc_detection_distance_;
    double mpc_opponent_timeout_sec_;
    double mpc_collision_weight_;
    double mpc_tracking_weight_;
    double mpc_offset_weight_;
    double mpc_steering_weight_;
    double mpc_speed_weight_;
    double mpc_overtake_speed_;
    double mpc_follow_speed_;
    double mpc_activation_margin_;
    double mpc_following_gap_;
    double mpc_follow_penalty_;
    double mpc_side_bias_weight_;
    std::vector<double> mpc_lateral_offsets_;
    std::string drive_topic_;
    std::string odom_topic_;
    std::string opponent_odom_topic_;
    std::string path_topic_;
    std::string base_frame_;
    std::string odom_frame_;
    std::string path_frame_;
    double timer_period_sec_ = 0.05;
    double marker_lifetime_sec_ = 0.20;

    // PID state
    double speed_error_integral_ = 0.0;
    double previous_speed_error_ = 0.0;
    bool have_previous_speed_error_ = false;
    rclcpp::Time last_control_time_;
    rclcpp::Time controller_start_time_;
    double previous_cmd_speed_ = 0.0;
    bool have_previous_cmd_speed_ = false;

    // Normalize angle to [-pi, pi]
    static double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Declare parameters
    void declare_parameters()
    {
        // Tunable controller parameters
        this->declare_parameter<double>("wheelbase", 0.3302);
        this->declare_parameter<double>("lookahead_distance", 1.0);
        this->declare_parameter<bool>("use_velocity_scaled_lookahead", true);
        this->declare_parameter<double>("min_lookahead_distance", 0.6);
        this->declare_parameter<double>("max_lookahead_distance", 2.0);
        this->declare_parameter<double>("lookahead_time", 0.8);
        this->declare_parameter<double>("speed", 2.0);
        this->declare_parameter<double>("reverse_speed", 2.0);
        this->declare_parameter<double>("max_steering_angle", 0.4);
        this->declare_parameter<double>("goal_tolerance", 0.2);
        this->declare_parameter<double>("timer_period_sec", 0.05);
        this->declare_parameter<double>("min_speed_near_goal", 0.4);
        this->declare_parameter<double>("sharp_turn_threshold", 0.25);
        this->declare_parameter<double>("goal_slowdown_distance", 1.0);
        this->declare_parameter<double>("turn_speed_reduction_factor", 0.6);
        this->declare_parameter<double>("straight_speed", 2.8);
        this->declare_parameter<double>("turn_speed", 1.15);
        this->declare_parameter<double>("close_obstacle_speed", 0.55);
        this->declare_parameter<double>("obstacle_slowdown_distance", 3.2);
        this->declare_parameter<double>("obstacle_stop_distance", 0.85);
        this->declare_parameter<double>("max_acceleration", 1.8);
        this->declare_parameter<double>("max_deceleration", 3.0);

        // Longitudinal PID
        this->declare_parameter<double>("pid_kp", 0.60);
        this->declare_parameter<double>("pid_ki", 0.05);
        this->declare_parameter<double>("pid_kd", 0.02);
        this->declare_parameter<double>("pid_integral_limit", 2.0);
        this->declare_parameter<double>("pid_output_limit", 1.0);
        this->declare_parameter<double>("max_speed_command", 3.0);
        this->declare_parameter<bool>("use_speed_pid", false);

        this->declare_parameter<bool>("enable_mpc_overtaking", true);
        this->declare_parameter<std::string>("opponent_odom_topic", "/opp_racecar/odom");
        this->declare_parameter<int>("mpc_horizon_steps", 14);
        this->declare_parameter<double>("mpc_dt", 0.12);
        this->declare_parameter<double>("mpc_safety_radius", 0.95);
        this->declare_parameter<double>("mpc_detection_distance", 5.0);
        this->declare_parameter<double>("mpc_opponent_timeout_sec", 0.6);
        this->declare_parameter<double>("mpc_collision_weight", 1200.0);
        this->declare_parameter<double>("mpc_tracking_weight", 3.0);
        this->declare_parameter<double>("mpc_offset_weight", 0.45);
        this->declare_parameter<double>("mpc_steering_weight", 1.0);
        this->declare_parameter<double>("mpc_speed_weight", 0.15);
        this->declare_parameter<double>("mpc_overtake_speed", 2.35);
        this->declare_parameter<double>("mpc_follow_speed", 0.75);
        this->declare_parameter<double>("mpc_activation_margin", 0.35);
        this->declare_parameter<double>("mpc_following_gap", 2.4);
        this->declare_parameter<double>("mpc_follow_penalty", 6.5);
        this->declare_parameter<double>("mpc_side_bias_weight", 4.0);
        this->declare_parameter<std::vector<double>>(
            "mpc_lateral_offsets",
            std::vector<double>{0.0, -0.45, 0.45, -0.75, 0.75, -1.05, 1.05});

        // RViz markers
        this->declare_parameter<double>("marker_lifetime_sec", 0.20);

        // Reverse/forward hysteresis
        this->declare_parameter<double>("reverse_trigger_x", -0.15);
        this->declare_parameter<double>("forward_trigger_x", 0.15);

        // Current f1tenth_gym_ros / Nav2 interface. Override these for hardware.
        this->declare_parameter<std::string>("drive_topic", "/drive");
        this->declare_parameter<std::string>("odom_topic", "/odometry/filtered");
        this->declare_parameter<std::string>("path_topic", "/control/plan");
        this->declare_parameter<std::string>("base_frame", "ego_racecar/base_link");
        this->declare_parameter<std::string>("odom_frame", "ego_racecar/odom");
    }

    // Read parameters once at startup
    void read_parameters()
    {
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        use_velocity_scaled_lookahead_ = this->get_parameter("use_velocity_scaled_lookahead").as_bool();
        min_lookahead_distance_ = this->get_parameter("min_lookahead_distance").as_double();
        max_lookahead_distance_ = this->get_parameter("max_lookahead_distance").as_double();
        lookahead_time_ = this->get_parameter("lookahead_time").as_double();
        speed_ = this->get_parameter("speed").as_double();
        reverse_speed_ = this->get_parameter("reverse_speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        min_speed_near_goal_ = this->get_parameter("min_speed_near_goal").as_double();
        sharp_turn_threshold_ = this->get_parameter("sharp_turn_threshold").as_double();
        goal_slowdown_distance_ = this->get_parameter("goal_slowdown_distance").as_double();
        turn_speed_reduction_factor_ = this->get_parameter("turn_speed_reduction_factor").as_double();
        straight_speed_ = this->get_parameter("straight_speed").as_double();
        turn_speed_ = this->get_parameter("turn_speed").as_double();
        close_obstacle_speed_ = this->get_parameter("close_obstacle_speed").as_double();
        obstacle_slowdown_distance_ = this->get_parameter("obstacle_slowdown_distance").as_double();
        obstacle_stop_distance_ = this->get_parameter("obstacle_stop_distance").as_double();
        max_acceleration_ = this->get_parameter("max_acceleration").as_double();
        max_deceleration_ = this->get_parameter("max_deceleration").as_double();
        pid_kp_ = this->get_parameter("pid_kp").as_double();
        pid_ki_ = this->get_parameter("pid_ki").as_double();
        pid_kd_ = this->get_parameter("pid_kd").as_double();
        pid_integral_limit_ = this->get_parameter("pid_integral_limit").as_double();
        pid_output_limit_ = this->get_parameter("pid_output_limit").as_double();
        max_speed_command_ = this->get_parameter("max_speed_command").as_double();
        use_speed_pid_ = this->get_parameter("use_speed_pid").as_bool();
        enable_mpc_overtaking_ = this->get_parameter("enable_mpc_overtaking").as_bool();
        opponent_odom_topic_ = this->get_parameter("opponent_odom_topic").as_string();
        mpc_horizon_steps_ = this->get_parameter("mpc_horizon_steps").as_int();
        mpc_dt_ = this->get_parameter("mpc_dt").as_double();
        mpc_safety_radius_ = this->get_parameter("mpc_safety_radius").as_double();
        mpc_detection_distance_ = this->get_parameter("mpc_detection_distance").as_double();
        mpc_opponent_timeout_sec_ = this->get_parameter("mpc_opponent_timeout_sec").as_double();
        mpc_collision_weight_ = this->get_parameter("mpc_collision_weight").as_double();
        mpc_tracking_weight_ = this->get_parameter("mpc_tracking_weight").as_double();
        mpc_offset_weight_ = this->get_parameter("mpc_offset_weight").as_double();
        mpc_steering_weight_ = this->get_parameter("mpc_steering_weight").as_double();
        mpc_speed_weight_ = this->get_parameter("mpc_speed_weight").as_double();
        mpc_overtake_speed_ = this->get_parameter("mpc_overtake_speed").as_double();
        mpc_follow_speed_ = this->get_parameter("mpc_follow_speed").as_double();
        mpc_activation_margin_ = this->get_parameter("mpc_activation_margin").as_double();
        mpc_following_gap_ = this->get_parameter("mpc_following_gap").as_double();
        mpc_follow_penalty_ = this->get_parameter("mpc_follow_penalty").as_double();
        mpc_side_bias_weight_ = this->get_parameter("mpc_side_bias_weight").as_double();
        mpc_lateral_offsets_ = this->get_parameter("mpc_lateral_offsets").as_double_array();
        reverse_trigger_x_ = this->get_parameter("reverse_trigger_x").as_double();
        forward_trigger_x_ = this->get_parameter("forward_trigger_x").as_double();
        marker_lifetime_sec_ = this->get_parameter("marker_lifetime_sec").as_double();
        drive_topic_ = this->get_parameter("drive_topic").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        path_topic_ = this->get_parameter("path_topic").as_string();
        base_frame_ = this->get_parameter("base_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();

        if (min_lookahead_distance_ <= 0.0 || max_lookahead_distance_ < min_lookahead_distance_)
        {
            throw std::invalid_argument(
                "lookahead distances must satisfy 0 < min_lookahead_distance <= max_lookahead_distance");
        }

        if (mpc_horizon_steps_ < 1)
            mpc_horizon_steps_ = 1;
        if (mpc_dt_ <= 1e-3)
            mpc_dt_ = 0.12;
        if (mpc_lateral_offsets_.empty())
            mpc_lateral_offsets_ = {0.0, -0.65, 0.65};

        timer_period_sec_ = this->get_parameter("timer_period_sec").as_double();
        last_control_time_ = this->now();
        controller_start_time_ = last_control_time_;
    }

    // Euclidean distance in 2D
    double distance_xy(double x1, double y1, double x2, double y2) const
    {
        const double dx = x2 - x1;
        const double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

    geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0) const
    {
        geometry_msgs::msg::Point p;
        p.x = x;
        p.y = y;
        p.z = z;
        return p;
    }

    visualization_msgs::msg::Marker make_base_marker(
        int id,
        int type,
        const std::string & ns,
        const std::string & frame_id,
        const rclcpp::Time & stamp) const
    {
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = frame_id;
        marker.header.stamp = stamp;
        marker.ns = ns;
        marker.id = id;
        marker.type = type;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;
        marker.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
        return marker;
    }

    double clamp_signed_speed(double speed) const
    {
        return std::clamp(speed, -max_speed_command_, max_speed_command_);
    }

    double current_lookahead_distance() const
    {
        if (!use_velocity_scaled_lookahead_)
            return lookahead_distance_;

        return std::clamp(
            std::abs(velocity_) * lookahead_time_,
            min_lookahead_distance_,
            max_lookahead_distance_);
    }

    void reset_speed_pid()
    {
        speed_error_integral_ = 0.0;
        previous_speed_error_ = 0.0;
        have_previous_speed_error_ = false;
        previous_cmd_speed_ = 0.0;
        have_previous_cmd_speed_ = false;
    }

    double rate_limit_speed(double target_speed, double dt)
    {
        if (!have_previous_cmd_speed_)
        {
            previous_cmd_speed_ = std::clamp(target_speed, -max_speed_command_, max_speed_command_);
            have_previous_cmd_speed_ = true;
            return previous_cmd_speed_;
        }

        const double delta = target_speed - previous_cmd_speed_;
        const double limit = delta >= 0.0
            ? std::max(0.0, max_acceleration_) * dt
            : std::max(0.0, max_deceleration_) * dt;

        previous_cmd_speed_ += std::clamp(delta, -limit, limit);
        previous_cmd_speed_ = std::clamp(previous_cmd_speed_, -max_speed_command_, max_speed_command_);
        return previous_cmd_speed_;
    }

    double compute_dt_seconds()
    {
        const rclcpp::Time now = this->now();
        double dt = (now - last_control_time_).seconds();
        last_control_time_ = now;

        if (dt <= 1e-4 || !std::isfinite(dt))
        {
            dt = timer_period_sec_;
        }

        return dt;
    }

    double compute_pid_speed_command(double target_speed, double dt)
    {
        const double speed_error = target_speed - velocity_;

        speed_error_integral_ += speed_error * dt;
        speed_error_integral_ = std::clamp(
            speed_error_integral_,
            -pid_integral_limit_,
            pid_integral_limit_
        );

        double derivative = 0.0;
        if (have_previous_speed_error_ && dt > 1e-4)
        {
            derivative = (speed_error - previous_speed_error_) / dt;
        }

        previous_speed_error_ = speed_error;
        have_previous_speed_error_ = true;

        const double pid_correction = std::clamp(
            pid_kp_ * speed_error +
            pid_ki_ * speed_error_integral_ +
            pid_kd_ * derivative,
            -pid_output_limit_,
            pid_output_limit_
        );

        return clamp_signed_speed(target_speed + pid_correction);
    }

    double opponent_forward_distance() const
    {
        if (!opponent_state_is_fresh())
            return std::numeric_limits<double>::infinity();

        const double dx = opponent_x_ - x_;
        const double dy = opponent_y_ - y_;
        const double forward = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
        const double lateral = std::abs(-std::sin(yaw_) * dx + std::cos(yaw_) * dy);

        if (forward < 0.0 || lateral > 1.5)
            return std::numeric_limits<double>::infinity();

        return forward;
    }

    double speed_for_steering(double requested_speed, double steering) const
    {
        const double abs_steer = std::abs(steering);
        if (abs_steer <= sharp_turn_threshold_)
            return requested_speed;

        const double ratio = std::clamp(
            (abs_steer - sharp_turn_threshold_) /
            std::max(1e-3, max_steering_angle_ - sharp_turn_threshold_),
            0.0,
            1.0);
        return requested_speed * (1.0 - ratio) + turn_speed_ * ratio;
    }

    double speed_for_obstacle(double requested_speed, const MpcDecision & decision) const
    {
        if (decision.active || !decision.opponent_relevant)
            return requested_speed;

        const double forward = opponent_forward_distance();
        if (!std::isfinite(forward) || forward >= obstacle_slowdown_distance_)
            return requested_speed;

        if (forward <= obstacle_stop_distance_)
            return std::min(requested_speed, close_obstacle_speed_);

        const double ratio = (forward - obstacle_stop_distance_) /
            std::max(1e-3, obstacle_slowdown_distance_ - obstacle_stop_distance_);
        const double capped_speed = close_obstacle_speed_ +
            ratio * (requested_speed - close_obstacle_speed_);
        return std::min(requested_speed, capped_speed);
    }

    double opponent_follow_penalty() const
    {
        if (!opponent_state_is_fresh())
            return 0.0;

        const double forward = opponent_forward_distance();
        if (!std::isfinite(forward) || forward >= mpc_following_gap_)
            return 0.0;

        const double gap = std::max(0.0, mpc_following_gap_ - forward);
        return mpc_follow_penalty_ * gap * gap;
    }

    double opponent_lateral_velocity() const
    {
        if (!opponent_state_is_fresh())
            return 0.0;

        const double opp_vx = opponent_speed_ * std::cos(opponent_yaw_);
        const double opp_vy = opponent_speed_ * std::sin(opponent_yaw_);
        return -std::sin(yaw_) * opp_vx + std::cos(yaw_) * opp_vy;
    }

    double preferred_overtake_side() const
    {
        if (!opponent_state_is_fresh())
            return 0.0;

        const double dx = opponent_x_ - x_;
        const double dy = opponent_y_ - y_;
        const double forward = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
        const double lateral = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
        if (forward < -0.2 || forward > mpc_detection_distance_)
            return 0.0;

        const double lateral_velocity = opponent_lateral_velocity();
        if (lateral_velocity > 0.15)
            return -1.0;
        if (lateral_velocity < -0.15)
            return 1.0;
        if (lateral > 0.15)
            return -1.0;
        if (lateral < -0.15)
            return 1.0;

        return 0.0;
    }

    bool compute_tracking_errors(TrackingErrors & errors) const
    {
        if (path_.size() < 2)
            return false;

        // IMPORTANT:
        // For error evaluation, search the whole path,
        // not only from last_progress_idx_ onward.
        int start_idx = 0;
        double best_dist_sq = std::numeric_limits<double>::max();
        bool found = false;

        for (int i = start_idx; i < static_cast<int>(path_.size()) - 1; ++i)
        {
            const double x1 = path_[i].x;
            const double y1 = path_[i].y;
            const double x2 = path_[i + 1].x;
            const double y2 = path_[i + 1].y;

            const double seg_dx = x2 - x1;
            const double seg_dy = y2 - y1;
            const double seg_len_sq = seg_dx * seg_dx + seg_dy * seg_dy;
            if (seg_len_sq < 1e-9)
                continue;

            const double rx = x_ - x1;
            const double ry = y_ - y1;
            const double t = std::clamp((rx * seg_dx + ry * seg_dy) / seg_len_sq, 0.0, 1.0);

            const double proj_x = x1 + t * seg_dx;
            const double proj_y = y1 + t * seg_dy;
            const double err_x = x_ - proj_x;
            const double err_y = y_ - proj_y;
            const double dist_sq = err_x * err_x + err_y * err_y;

            if (dist_sq < best_dist_sq)
            {
                best_dist_sq = dist_sq;
                errors.proj_x = proj_x;
                errors.proj_y = proj_y;
                errors.path_yaw = std::atan2(seg_dy, seg_dx);
                found = true;
            }
        }

        if (!found)
            return false;

        const double dx_local = errors.proj_x - x_;
        const double dy_local = errors.proj_y - y_;

        // In the robot frame, +y means path is to the left of the vehicle.
        const double proj_y_local = -std::sin(yaw_) * dx_local + std::cos(yaw_) * dy_local;
        errors.cross_track_error = proj_y_local;

        double reference_yaw = errors.path_yaw;
        if (reverse_mode_)
        {
            reference_yaw = normalize_angle(reference_yaw + M_PI);
        }
        errors.heading_error = normalize_angle(reference_yaw - yaw_);
        return true;
    }

    void publish_tracking_errors(const TrackingErrors & tracking_errors) const
    {
        geometry_msgs::msg::Vector3Stamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = base_frame_;
        msg.vector.x = tracking_errors.cross_track_error;
        msg.vector.y = tracking_errors.heading_error;
        msg.vector.z = 0.0;
        tracking_error_pub_->publish(msg);
    }

    void publish_debug_markers(double target_x,
                               double target_y,
                               double steering,
                               bool have_tracking_errors,
                               const TrackingErrors & tracking_errors)
    {
        const rclcpp::Time now = this->now();
        visualization_msgs::msg::MarkerArray array;

        // 1) Target point marker
        const std::string & marker_frame = path_frame_.empty() ? odom_frame_ : path_frame_;
        const double lookahead = current_lookahead_distance();

        auto target_marker = make_base_marker(0, visualization_msgs::msg::Marker::SPHERE,
                                              "pure_pursuit", marker_frame, now);
        target_marker.pose.position = make_point(target_x, target_y, 0.05);
        target_marker.scale.x = 0.18;
        target_marker.scale.y = 0.18;
        target_marker.scale.z = 0.18;
        target_marker.color.a = 1.0;
        target_marker.color.r = 1.0;
        target_marker.color.g = 0.2;
        target_marker.color.b = 0.2;
        array.markers.push_back(target_marker);

        // 2) Lookahead circle marker
        auto circle_marker = make_base_marker(1, visualization_msgs::msg::Marker::LINE_STRIP,
                                              "pure_pursuit", marker_frame, now);
        circle_marker.scale.x = 0.03;
        circle_marker.color.a = 0.95;
        circle_marker.color.r = 0.2;
        circle_marker.color.g = 0.6;
        circle_marker.color.b = 1.0;
        constexpr int kCircleSamples = 48;
        for (int i = 0; i <= kCircleSamples; ++i)
        {
            const double theta = 2.0 * M_PI * static_cast<double>(i) / static_cast<double>(kCircleSamples);
            circle_marker.points.push_back(make_point(
                x_ + lookahead * std::cos(theta),
                y_ + lookahead * std::sin(theta),
                0.02));
        }
        array.markers.push_back(circle_marker);

        // 3) Error vector marker (robot -> path projection)
        auto error_marker = make_base_marker(2, visualization_msgs::msg::Marker::LINE_STRIP,
                                             "pure_pursuit", marker_frame, now);
        error_marker.scale.x = 0.05;
        error_marker.color.a = 1.0;
        error_marker.color.r = 1.0;
        error_marker.color.g = 0.9;
        error_marker.color.b = 0.1;
        if (have_tracking_errors)
        {
            error_marker.points.push_back(make_point(x_, y_, 0.04));
            error_marker.points.push_back(make_point(tracking_errors.proj_x, tracking_errors.proj_y, 0.04));
        }
        array.markers.push_back(error_marker);

        // 4) Steering direction marker
        auto steering_marker = make_base_marker(3, visualization_msgs::msg::Marker::ARROW,
                                                "pure_pursuit", marker_frame, now);

        // arrow size
        steering_marker.scale.x = 0.04;   // shaft diameter
        steering_marker.scale.y = 0.05;    // head diameter
        steering_marker.scale.z = 0.07;    // head length

        // green color
        steering_marker.color.a = 1.0;
        steering_marker.color.r = 0.1;
        steering_marker.color.g = 1.0;
        steering_marker.color.b = 0.2;

        // shorter arrow
        const double arrow_length = 0.30;
        const double motion_yaw = reverse_mode_ ? normalize_angle(yaw_ + M_PI) : yaw_;
        const double steer_yaw = motion_yaw + steering;

        steering_marker.points.clear();
        steering_marker.points.push_back(make_point(x_, y_, 0.06));
        steering_marker.points.push_back(make_point(
            x_ + arrow_length * std::cos(steer_yaw),
            y_ + arrow_length * std::sin(steer_yaw),
            0.06));

        array.markers.push_back(steering_marker);

        debug_marker_pub_->publish(array);
    }

    // Publish zero speed and zero steering
    void publish_stop()
    {
        reset_speed_pid();

        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = base_frame_;
        msg.drive.speed = 0.0;
        msg.drive.steering_angle = 0.0;
        nav_pub_->publish(msg);
    }

    // -------------------------------------------------
    // Odom callback:
    // reads current robot position, heading, and velocity
    // -------------------------------------------------
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_ = msg->pose.pose.position.x;
        y_ = msg->pose.pose.position.y;
        velocity_ = msg->twist.twist.linear.x;

        auto q = msg->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        yaw_ = yaw;

        have_odom_ = true;
    }

    void opponent_odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        opponent_x_ = msg->pose.pose.position.x;
        opponent_y_ = msg->pose.pose.position.y;
        opponent_speed_ = msg->twist.twist.linear.x;

        auto q = msg->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);

        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        opponent_yaw_ = yaw;

        last_opponent_stamp_ = this->now();
        have_opponent_ = true;
    }

    // -------------------------------------------------
    // Path callback:
    // stores the newest global path from /plan
    // -------------------------------------------------
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        const rclcpp::Time path_stamp(msg->header.stamp);
        if (path_stamp.nanoseconds() > 0 && path_stamp < controller_start_time_)
        {
            RCLCPP_WARN(this->get_logger(),
                        "Ignoring stale retained path on %s. path_stamp=%.3f controller_start=%.3f",
                        path_topic_.c_str(),
                        path_stamp.seconds(),
                        controller_start_time_.seconds());
            return;
        }

        path_.clear();
        path_frame_ = msg->header.frame_id;

        if (msg->poses.empty())
        {
            have_path_ = false;
            goal_reached_ = false;
            reverse_mode_ = false;
            last_progress_idx_ = 0;
            reset_speed_pid();
            RCLCPP_WARN(this->get_logger(), "Received empty path on %s.", path_topic_.c_str());
            return;
        }

        path_.reserve(msg->poses.size());

        for (const auto & pose_stamped : msg->poses)
        {
            Point2D p;
            p.x = pose_stamped.pose.position.x;
            p.y = pose_stamped.pose.position.y;
            path_.push_back(p);
        }

        have_path_ = true;
        goal_reached_ = false;
        reverse_mode_ = false;
        last_progress_idx_ = 0;
        reset_speed_pid();

        RCLCPP_INFO(this->get_logger(),
                    "Received path with %zu points in frame '%s'. Start=(%.2f, %.2f) End=(%.2f, %.2f)",
                    path_.size(),
                    path_frame_.c_str(),
                    path_.front().x, path_.front().y,
                    path_.back().x, path_.back().y);
    }

    // -------------------------------------------------
    // Find nearest point on path, but only forward from the
    // current progress index. This avoids jumping backward.
    // -------------------------------------------------
    int find_nearest_forward_index()
    {
        if (path_.empty())
            return 0;

        int start_idx = std::max(
            0,
            std::min(last_progress_idx_, static_cast<int>(path_.size()) - 1)
        );

        int nearest_idx = start_idx;
        double min_dist = std::numeric_limits<double>::max();

        for (int i = start_idx; i < static_cast<int>(path_.size()); ++i)
        {
            double d = distance_xy(x_, y_, path_[i].x, path_[i].y);
            if (d < min_dist)
            {
                min_dist = d;
                nearest_idx = i;
            }
        }

        return nearest_idx;
    }

    // ---------------------------------------------------------
    // Lookahead option 1: waypoint-based
    // Difference:
    // Uses the first original path point farther than lookahead.
    // ---------------------------------------------------------
    bool find_lookahead_target_waypoint(int start_idx,
                                        double &target_x,
                                        double &target_y,
                                        int &target_idx,
                                        double lookahead)
    {
        if (path_.empty())
            return false;

        start_idx = std::max(
            0,
            std::min(start_idx, static_cast<int>(path_.size()) - 1)
        );

        for (int i = start_idx; i < static_cast<int>(path_.size()); ++i)
        {
            double d = distance_xy(x_, y_, path_[i].x, path_[i].y);

            if (d >= lookahead)
            {
                target_x = path_[i].x;
                target_y = path_[i].y;
                target_idx = i;
                return true;
            }
        }

        // Fallback to final goal point
        target_x = path_.back().x;
        target_y = path_.back().y;
        target_idx = static_cast<int>(path_.size()) - 1;
        return true;
    }

    // ---------------------------------------------------------
    // Lookahead option 2: segment-based
    // Difference:
    // Creates an interpolated point on the segment-circle intersection.
    // ---------------------------------------------------------
    bool find_lookahead_target_segment(int start_idx,
                                       double &target_x,
                                       double &target_y,
                                       int &target_idx,
                                       double lookahead)
    {
        if (path_.empty())
            return false;

        if (path_.size() == 1)
        {
            target_x = path_[0].x;
            target_y = path_[0].y;
            target_idx = 0;
            return true;
        }

        start_idx = std::max(
            0,
            std::min(start_idx, static_cast<int>(path_.size()) - 2)
        );

        for (int i = start_idx; i < static_cast<int>(path_.size()) - 1; ++i)
        {
            const double x1 = path_[i].x;
            const double y1 = path_[i].y;
            const double x2 = path_[i + 1].x;
            const double y2 = path_[i + 1].y;

            const double dx = x2 - x1;
            const double dy = y2 - y1;

            const double fx = x1 - x_;
            const double fy = y1 - y_;

            const double a = dx * dx + dy * dy;
            if (a < 1e-9)
                continue;

            const double b = 2.0 * (fx * dx + fy * dy);
            const double c = fx * fx + fy * fy - lookahead * lookahead;

            const double discriminant = b * b - 4.0 * a * c;
            if (discriminant < 0.0)
                continue;

            const double sqrt_disc = std::sqrt(discriminant);
            const double t1 = (-b - sqrt_disc) / (2.0 * a);
            const double t2 = (-b + sqrt_disc) / (2.0 * a);

            std::vector<double> candidates;
            if (t1 >= 0.0 && t1 <= 1.0) candidates.push_back(t1);
            if (t2 >= 0.0 && t2 <= 1.0) candidates.push_back(t2);

            if (!candidates.empty())
            {
                std::sort(candidates.begin(), candidates.end());
                const double t = candidates.front();

                target_x = x1 + t * dx;
                target_y = y1 + t * dy;
                target_idx = i;
                return true;
            }
        }

        // Fallback to final goal point
        target_x = path_.back().x;
        target_y = path_.back().y;
        target_idx = static_cast<int>(path_.size()) - 1;
        return true;
    }

    bool opponent_state_is_fresh() const
    {
        if (!have_opponent_)
            return false;

        const double age = (this->now() - last_opponent_stamp_).seconds();
        return std::isfinite(age) && age <= mpc_opponent_timeout_sec_;
    }

    double path_yaw_at_index(int index) const
    {
        if (path_.size() < 2)
            return yaw_;

        index = std::clamp(index, 0, static_cast<int>(path_.size()) - 1);
        int next = std::min(index + 1, static_cast<int>(path_.size()) - 1);
        int prev = std::max(index - 1, 0);

        if (next == index && prev != index)
            next = index;
        if (prev == next)
            return yaw_;

        return std::atan2(path_[next].y - path_[prev].y, path_[next].x - path_[prev].x);
    }

    double signed_lateral_to_path(double px, double py, int path_index) const
    {
        if (path_.empty())
            return 0.0;

        path_index = std::clamp(path_index, 0, static_cast<int>(path_.size()) - 1);
        const double path_yaw = path_yaw_at_index(path_index);
        const double dx = px - path_[path_index].x;
        const double dy = py - path_[path_index].y;
        return -std::sin(path_yaw) * dx + std::cos(path_yaw) * dy;
    }

    bool opponent_is_relevant(int nearest_idx) const
    {
        if (!enable_mpc_overtaking_ || !opponent_state_is_fresh() || path_.empty())
            return false;

        const double dx = opponent_x_ - x_;
        const double dy = opponent_y_ - y_;
        const double forward = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
        const double lateral = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;

        if (forward < -0.5 || forward > mpc_detection_distance_)
            return false;

        const double path_lateral = std::abs(signed_lateral_to_path(
            opponent_x_, opponent_y_, nearest_idx));

        return std::abs(lateral) < 2.2 || path_lateral < 1.8;
    }

    void shifted_target(double base_x,
                        double base_y,
                        int target_idx,
                        double lateral_offset,
                        double &shifted_x,
                        double &shifted_y) const
    {
        const double path_yaw = path_yaw_at_index(target_idx);
        shifted_x = base_x - std::sin(path_yaw) * lateral_offset;
        shifted_y = base_y + std::cos(path_yaw) * lateral_offset;
    }

    double steering_to_target(double tx, double ty) const
    {
        const double dx = tx - x_;
        const double dy = ty - y_;
        const double x_local = std::cos(yaw_) * dx + std::sin(yaw_) * dy;
        const double y_local = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;
        const double alpha = normalize_angle(std::atan2(y_local, std::max(1e-3, x_local)));
        double ld = std::sqrt(dx * dx + dy * dy);
        if (ld < 1e-3)
            ld = 1e-3;

        return std::clamp(
            std::atan2(2.0 * wheelbase_ * std::sin(alpha), ld),
            -max_steering_angle_,
            max_steering_angle_);
    }

    double rollout_candidate_cost(double target_x,
                                  double target_y,
                                  double lateral_offset,
                                  double candidate_speed,
                                  double steering,
                                  double &min_clearance) const
    {
        double ego_x = x_;
        double ego_y = y_;
        double ego_yaw = yaw_;
        double opp_x = opponent_x_;
        double opp_y = opponent_y_;

        const double opp_vx = opponent_speed_ * std::cos(opponent_yaw_);
        const double opp_vy = opponent_speed_ * std::sin(opponent_yaw_);

        double cost = 0.0;
        min_clearance = std::numeric_limits<double>::infinity();

        for (int i = 1; i <= mpc_horizon_steps_; ++i)
        {
            ego_x += candidate_speed * std::cos(ego_yaw) * mpc_dt_;
            ego_y += candidate_speed * std::sin(ego_yaw) * mpc_dt_;
            ego_yaw = normalize_angle(
                ego_yaw + (candidate_speed / wheelbase_) * std::tan(steering) * mpc_dt_);

            opp_x += opp_vx * mpc_dt_;
            opp_y += opp_vy * mpc_dt_;

            const double clearance = distance_xy(ego_x, ego_y, opp_x, opp_y);
            min_clearance = std::min(min_clearance, clearance);

            if (clearance < mpc_safety_radius_)
            {
                const double violation = mpc_safety_radius_ - clearance;
                cost += mpc_collision_weight_ * violation * violation;
            }
            else
            {
                cost += 1.0 / std::max(0.10, clearance - mpc_safety_radius_);
            }
        }

        const double terminal_error = distance_xy(ego_x, ego_y, target_x, target_y);
        cost += mpc_tracking_weight_ * terminal_error * terminal_error;
        cost += mpc_offset_weight_ * std::abs(lateral_offset);
        cost += mpc_steering_weight_ * std::abs(steering);
        cost += mpc_speed_weight_ * std::abs(straight_speed_ - candidate_speed);

        return cost;
    }

    MpcDecision compute_mpc_overtake_decision(double base_target_x,
                                              double base_target_y,
                                              int target_idx,
                                              int nearest_idx)
    {
        MpcDecision decision;
        decision.target_x = base_target_x;
        decision.target_y = base_target_y;
        decision.target_speed = std::min(straight_speed_, max_speed_command_);
        decision.opponent_relevant = opponent_is_relevant(nearest_idx);

        if (!decision.opponent_relevant)
        {
            mpc_overtake_active_ = false;
            return decision;
        }

        double straight_x = base_target_x;
        double straight_y = base_target_y;
        shifted_target(base_target_x, base_target_y, target_idx, 0.0, straight_x, straight_y);
        double straight_clearance = 0.0;
        const double straight_steer = steering_to_target(straight_x, straight_y);
        const double straight_candidate_speed = std::min(straight_speed_, max_speed_command_);
        const double straight_cost = rollout_candidate_cost(
            straight_x, straight_y, 0.0, straight_candidate_speed, straight_steer, straight_clearance);

        double best_cost = straight_cost;
        double best_clearance = straight_clearance;
        double best_x = straight_x;
        double best_y = straight_y;
        double best_offset = 0.0;
        double best_speed = straight_candidate_speed;

        const double follow_penalty = opponent_follow_penalty();
        const double adjusted_straight_cost = straight_cost + follow_penalty;
        const double preferred_side = preferred_overtake_side();

        for (const double offset : mpc_lateral_offsets_)
        {
            double tx = base_target_x;
            double ty = base_target_y;
            shifted_target(base_target_x, base_target_y, target_idx, offset, tx, ty);

            const double candidate_speed = std::abs(offset) > 1e-3
                ? std::min(mpc_overtake_speed_, max_speed_command_)
                : std::min(mpc_follow_speed_, max_speed_command_);
            const double steering = steering_to_target(tx, ty);

            double min_clearance = 0.0;
            const double cost = rollout_candidate_cost(
                tx, ty, offset, candidate_speed, steering, min_clearance);

            double side_bias = 0.0;
            if (std::abs(offset) > 1e-3 && preferred_side != 0.0)
            {
                side_bias = (offset * preferred_side > 0.0)
                    ? -mpc_side_bias_weight_
                    : mpc_side_bias_weight_;
            }

            const double biased_cost = cost + side_bias;

            if (biased_cost < best_cost)
            {
                best_cost = biased_cost;
                best_clearance = min_clearance;
                best_x = tx;
                best_y = ty;
                best_offset = offset;
                best_speed = candidate_speed;
            }
        }

        const bool useful_offset = std::abs(best_offset) > 1e-3 &&
            best_cost < (adjusted_straight_cost - mpc_activation_margin_);
        const bool corridor_pressure = follow_penalty > 0.0 ||
            straight_clearance < (mpc_safety_radius_ + mpc_activation_margin_);
        mpc_overtake_active_ = corridor_pressure && useful_offset;

        decision.active = mpc_overtake_active_;
        decision.target_x = mpc_overtake_active_ ? best_x : base_target_x;
        decision.target_y = mpc_overtake_active_ ? best_y : base_target_y;
        decision.target_speed = mpc_overtake_active_
            ? best_speed
            : std::min(straight_speed_, max_speed_command_);
        decision.lateral_offset = mpc_overtake_active_ ? best_offset : 0.0;
        decision.cost = best_cost;
        decision.min_predicted_clearance = best_clearance;
        return decision;
    }

    // -------------------------------------------------
    // Main controller loop
    // -------------------------------------------------
    void timer_callback()
    {
        if (!have_odom_ || !have_path_ || path_.empty())
            return;

        // Keep stopping after goal is reached
        if (goal_reached_)
        {
            publish_stop();
            return;
        }

        // Final goal = last point in path
        const double goal_x = path_.back().x;
        const double goal_y = path_.back().y;
        const double dist_to_goal = distance_xy(x_, y_, goal_x, goal_y);

        if (dist_to_goal < goal_tolerance_)
        {
            goal_reached_ = true;
            reverse_mode_ = false;
            publish_stop();

            RCLCPP_INFO(this->get_logger(),
                        "Goal reached. x=%.2f y=%.2f goal=(%.2f, %.2f) dist=%.3f",
                        x_, y_, goal_x, goal_y, dist_to_goal);
            return;
        }

        // Update progress on the path
        int nearest_idx = find_nearest_forward_index();
        if (nearest_idx > last_progress_idx_)
            last_progress_idx_ = nearest_idx;

        // Default target = final goal
        double target_x = goal_x;
        double target_y = goal_y;
        int target_idx = last_progress_idx_;

        // =====================================================
        // Choose ONE lookahead strategy here
        // =====================================================

        // Option A: waypoint-based lookahead
        // bool ok = find_lookahead_target_waypoint(
        //     last_progress_idx_, target_x, target_y, target_idx, lookahead
        // );

        // Option B: segment-based lookahead
        const double lookahead = current_lookahead_distance();
        bool ok = find_lookahead_target_segment(
            last_progress_idx_, target_x, target_y, target_idx, lookahead
        );

        if (!ok)
            return;

        if (target_idx > last_progress_idx_)
            last_progress_idx_ = target_idx;

        const MpcDecision mpc_decision = compute_mpc_overtake_decision(
            target_x, target_y, target_idx, nearest_idx);
        target_x = mpc_decision.target_x;
        target_y = mpc_decision.target_y;

        // Vector from robot to target in world frame
        const double dx = target_x - x_;
        const double dy = target_y - y_;

        // Transform target into robot local frame
        // x_local > 0  -> target in front
        // x_local < 0  -> target behind
        const double x_local =  std::cos(yaw_) * dx + std::sin(yaw_) * dy;
        const double y_local = -std::sin(yaw_) * dx + std::cos(yaw_) * dy;

        // Hysteresis to avoid mode switching noise
        if (!reverse_mode_ && x_local < reverse_trigger_x_)
        {
            reverse_mode_ = true;
        }
        else if (reverse_mode_ && x_local > forward_trigger_x_)
        {
            reverse_mode_ = false;
        }

        const double dt = compute_dt_seconds();

        double alpha = 0.0;
        double target_speed = 0.0;

        // -------------------------------------------------
        // Forward mode
        // -------------------------------------------------
        if (!reverse_mode_)
        {
            // Target angle relative to front-driving direction
            alpha = std::atan2(y_local, x_local);
            target_speed = mpc_decision.target_speed;
        }
        // -------------------------------------------------
        // Reverse mode
        // -------------------------------------------------
        else
        {
            // In reverse, target is handled relative to rear-driving direction
            alpha = std::atan2(y_local, -x_local);
            target_speed = -reverse_speed_;
        }

        alpha = normalize_angle(alpha);

        // Actual robot-to-target distance
        double ld_actual = std::sqrt(dx * dx + dy * dy);
        if (ld_actual < 1e-3)
            ld_actual = 1e-3;

        // Pure pursuit steering law
        double steering = std::atan2(
            2.0 * wheelbase_ * std::sin(alpha),
            ld_actual
        );

        // Clamp steering to vehicle limits
        if (steering > max_steering_angle_) steering = max_steering_angle_;
        if (steering < -max_steering_angle_) steering = -max_steering_angle_;

        if (!reverse_mode_)
        {
            target_speed = speed_for_steering(target_speed, steering);
            target_speed = speed_for_obstacle(target_speed, mpc_decision);
        }
        else if (std::abs(steering) > sharp_turn_threshold_)
        {
            target_speed *= turn_speed_reduction_factor_;
        }

        // Slow down near goal while keeping the sign of speed
        if (dist_to_goal < goal_slowdown_distance_)
        {
            double limited_speed = std::min(
                std::abs(target_speed),
                std::max(min_speed_near_goal_, dist_to_goal)
            );

            target_speed = (target_speed >= 0.0) ? limited_speed : -limited_speed;
        }

        // Closed-loop longitudinal control: follow target_speed using odometry feedback
        const double unconstrained_cmd_speed = use_speed_pid_
            ? compute_pid_speed_command(target_speed, dt)
            : clamp_signed_speed(target_speed);
        const double cmd_speed = rate_limit_speed(unconstrained_cmd_speed, dt);

        // Publish control command
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = base_frame_;
        msg.drive.speed = cmd_speed;
        msg.drive.steering_angle = steering;
        nav_pub_->publish(msg);

        TrackingErrors tracking_errors;
        const bool have_tracking_errors = compute_tracking_errors(tracking_errors);
        if (have_tracking_errors) {
            publish_tracking_errors(tracking_errors);
        }
        publish_debug_markers(target_x, target_y, steering, have_tracking_errors, tracking_errors);

        const double speed_error = target_speed - velocity_;

        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                             "\n[Robot] mode=%s x=%.2f y=%.2f yaw=%.2f | "
                             "\n[Path] nearest=%d progress=%d target_idx=%d target=(%.2f, %.2f) x_local=%.2f y_local=%.2f dist_goal=%.2f | "
                             "\n[MPC] active=%s offset=%.2f clearance=%.2f cost=%.1f | "
                             "\n[Errors] cte=%.3f heading_err=%.3f alpha=%.2f | "
                             "\n[Control] steer=%.2f v_ref=%.2f v_meas=%.2f v_err=%.2f v_cmd=%.2f",
                             reverse_mode_ ? "REVERSE" : "FORWARD",
                             x_, y_, yaw_,
                             nearest_idx, last_progress_idx_, target_idx,
                             target_x, target_y,
                             x_local, y_local,
                             dist_to_goal,
                             mpc_decision.active ? "yes" : "no",
                             mpc_decision.lateral_offset,
                             mpc_decision.min_predicted_clearance,
                             mpc_decision.cost,
                             have_tracking_errors ? tracking_errors.cross_track_error : 0.0,
                             have_tracking_errors ? tracking_errors.heading_error : 0.0,
                             alpha,
                             steering,
                             target_speed, velocity_, speed_error, cmd_speed);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}
