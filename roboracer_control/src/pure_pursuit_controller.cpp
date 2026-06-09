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
                    "PurePursuitController: odom=%s path=%s drive=%s",
                    odom_topic_.c_str(), path_topic_.c_str(), drive_topic_.c_str());
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

    // -------------------------------------------------
    // ROS interfaces
    // -------------------------------------------------
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr nav_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr tracking_error_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
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
    std::string drive_topic_;
    std::string odom_topic_;
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

        // Longitudinal PID
        this->declare_parameter<double>("pid_kp", 0.60);
        this->declare_parameter<double>("pid_ki", 0.05);
        this->declare_parameter<double>("pid_kd", 0.02);
        this->declare_parameter<double>("pid_integral_limit", 2.0);
        this->declare_parameter<double>("pid_output_limit", 1.0);
        this->declare_parameter<double>("max_speed_command", 3.0);
        this->declare_parameter<bool>("use_speed_pid", false);

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
        pid_kp_ = this->get_parameter("pid_kp").as_double();
        pid_ki_ = this->get_parameter("pid_ki").as_double();
        pid_kd_ = this->get_parameter("pid_kd").as_double();
        pid_integral_limit_ = this->get_parameter("pid_integral_limit").as_double();
        pid_output_limit_ = this->get_parameter("pid_output_limit").as_double();
        max_speed_command_ = this->get_parameter("max_speed_command").as_double();
        use_speed_pid_ = this->get_parameter("use_speed_pid").as_bool();
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

        timer_period_sec_ = this->get_parameter("timer_period_sec").as_double();
        last_control_time_ = this->now();
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

    // -------------------------------------------------
    // Path callback:
    // stores the newest global path from /plan
    // -------------------------------------------------
    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
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
            target_speed = speed_;
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

        // Build a simple speed reference profile before closed-loop tracking
        if (std::abs(steering) > sharp_turn_threshold_)
            target_speed *= turn_speed_reduction_factor_;

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
        const double cmd_speed = use_speed_pid_
            ? compute_pid_speed_command(target_speed, dt)
            : clamp_signed_speed(target_speed);

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
                             "\n[Errors] cte=%.3f heading_err=%.3f alpha=%.2f | "
                             "\n[Control] steer=%.2f v_ref=%.2f v_meas=%.2f v_err=%.2f v_cmd=%.2f",
                             reverse_mode_ ? "REVERSE" : "FORWARD",
                             x_, y_, yaw_,
                             nearest_idx, last_progress_idx_, target_idx,
                             target_x, target_y,
                             x_local, y_local,
                             dist_to_goal,
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
