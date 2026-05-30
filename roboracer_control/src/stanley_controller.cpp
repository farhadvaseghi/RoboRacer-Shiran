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

using namespace std::chrono_literals;

class StanleyController : public rclcpp::Node
{
public:
    StanleyController() : Node("stanley_controller")
    {
        declare_parameters();
        read_parameters();

        nav_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/nav", 10);
        debug_marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/control_debug_markers", 10);
        tracking_error_pub_ = create_publisher<geometry_msgs::msg::Vector3Stamped>("/tracking_error", 10);

        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10,
            std::bind(&StanleyController::odom_callback, this, std::placeholders::_1));

        plan_sub_ = create_subscription<nav_msgs::msg::Path>(
            "/plan", 10,
            std::bind(&StanleyController::path_callback, this, std::placeholders::_1));

        timer_ = create_wall_timer(
            std::chrono::duration<double>(timer_period_sec_),
            std::bind(&StanleyController::timer_callback, this));

        RCLCPP_INFO(get_logger(), "StanleyController initialized. Waiting for /plan ...");
    }

private:
    struct Point2D { double x, y; };

    struct StanleyErrors {
        double cte = 0.0;           // cross-track error — positive when path is left of front axle
        double heading_error = 0.0; // normalize(path_yaw - vehicle_yaw)
        double proj_x = 0.0;
        double proj_y = 0.0;
    };

    // ROS interfaces
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr nav_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr debug_marker_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr tracking_error_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Robot state
    double x_ = 0.0, y_ = 0.0, yaw_ = 0.0, velocity_ = 0.0;
    bool have_odom_ = false;

    // Path state
    std::vector<Point2D> path_;
    bool have_path_ = false;
    bool goal_reached_ = false;
    int last_progress_idx_ = 0;

    // Parameters
    double wheelbase_;
    double stanley_k_;
    double stanley_k_soft_;
    double speed_;
    double max_steering_angle_;
    double goal_tolerance_;
    double goal_slowdown_distance_;
    double min_speed_near_goal_;
    double timer_period_sec_;
    double marker_lifetime_sec_;
    double pid_kp_, pid_ki_, pid_kd_;
    double pid_integral_limit_, pid_output_limit_;
    double max_speed_command_;

    // PID state
    double speed_error_integral_ = 0.0;
    double previous_speed_error_ = 0.0;
    bool have_previous_speed_error_ = false;
    rclcpp::Time last_control_time_;

    static double normalize_angle(double a)
    {
        while (a >  M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    }

    void declare_parameters()
    {
        declare_parameter<double>("wheelbase",             0.3302);
        declare_parameter<double>("stanley_k",             1.0);
        declare_parameter<double>("stanley_k_soft",        0.5);
        declare_parameter<double>("speed",                 2.0);
        declare_parameter<double>("max_steering_angle",    0.4);
        declare_parameter<double>("goal_tolerance",        0.2);
        declare_parameter<double>("goal_slowdown_distance",1.0);
        declare_parameter<double>("min_speed_near_goal",   0.4);
        declare_parameter<double>("timer_period_sec",      0.05);
        declare_parameter<double>("marker_lifetime_sec",   0.20);
        declare_parameter<double>("pid_kp",                0.60);
        declare_parameter<double>("pid_ki",                0.05);
        declare_parameter<double>("pid_kd",                0.02);
        declare_parameter<double>("pid_integral_limit",    2.0);
        declare_parameter<double>("pid_output_limit",      1.0);
        declare_parameter<double>("max_speed_command",     3.0);
    }

    void read_parameters()
    {
        wheelbase_             = get_parameter("wheelbase").as_double();
        stanley_k_             = get_parameter("stanley_k").as_double();
        stanley_k_soft_        = get_parameter("stanley_k_soft").as_double();
        speed_                 = get_parameter("speed").as_double();
        max_steering_angle_    = get_parameter("max_steering_angle").as_double();
        goal_tolerance_        = get_parameter("goal_tolerance").as_double();
        goal_slowdown_distance_= get_parameter("goal_slowdown_distance").as_double();
        min_speed_near_goal_   = get_parameter("min_speed_near_goal").as_double();
        timer_period_sec_      = get_parameter("timer_period_sec").as_double();
        marker_lifetime_sec_   = get_parameter("marker_lifetime_sec").as_double();
        pid_kp_                = get_parameter("pid_kp").as_double();
        pid_ki_                = get_parameter("pid_ki").as_double();
        pid_kd_                = get_parameter("pid_kd").as_double();
        pid_integral_limit_    = get_parameter("pid_integral_limit").as_double();
        pid_output_limit_      = get_parameter("pid_output_limit").as_double();
        max_speed_command_     = get_parameter("max_speed_command").as_double();
        last_control_time_     = now();
    }

    double distance_xy(double x1, double y1, double x2, double y2) const
    {
        return std::sqrt((x2-x1)*(x2-x1) + (y2-y1)*(y2-y1));
    }

    void reset_speed_pid()
    {
        speed_error_integral_ = 0.0;
        previous_speed_error_ = 0.0;
        have_previous_speed_error_ = false;
    }

    double compute_dt_seconds()
    {
        const rclcpp::Time t = now();
        double dt = (t - last_control_time_).seconds();
        last_control_time_ = t;
        if (dt <= 1e-4 || !std::isfinite(dt)) dt = timer_period_sec_;
        return dt;
    }

    double compute_pid_speed_command(double target_speed, double dt)
    {
        const double error = target_speed - velocity_;
        speed_error_integral_ = std::clamp(
            speed_error_integral_ + error * dt,
            -pid_integral_limit_, pid_integral_limit_);

        double derivative = 0.0;
        if (have_previous_speed_error_ && dt > 1e-4)
            derivative = (error - previous_speed_error_) / dt;

        previous_speed_error_ = error;
        have_previous_speed_error_ = true;

        const double correction = std::clamp(
            pid_kp_ * error + pid_ki_ * speed_error_integral_ + pid_kd_ * derivative,
            -pid_output_limit_, pid_output_limit_);

        return std::clamp(target_speed + correction, -max_speed_command_, max_speed_command_);
    }

    // Project the front axle onto every path segment and return the errors
    // for the nearest one. Searching forward from last_progress_idx_ with a
    // small lookback so the front axle never snaps to a segment behind the car.
    bool compute_stanley_errors(double front_x, double front_y, StanleyErrors & out) const
    {
        if (path_.size() < 2) return false;

        const int start = std::max(0, last_progress_idx_ - 5);
        double best_dist_sq = std::numeric_limits<double>::max();
        bool found = false;

        for (int i = start; i < static_cast<int>(path_.size()) - 1; ++i)
        {
            const double x1 = path_[i].x,   y1 = path_[i].y;
            const double x2 = path_[i+1].x, y2 = path_[i+1].y;
            const double seg_dx = x2 - x1,  seg_dy = y2 - y1;
            const double seg_len_sq = seg_dx*seg_dx + seg_dy*seg_dy;
            if (seg_len_sq < 1e-9) continue;

            const double t = std::clamp(
                ((front_x - x1) * seg_dx + (front_y - y1) * seg_dy) / seg_len_sq,
                0.0, 1.0);

            const double px = x1 + t * seg_dx;
            const double py = y1 + t * seg_dy;
            const double ex = front_x - px;
            const double ey = front_y - py;
            const double dist_sq = ex*ex + ey*ey;

            if (dist_sq < best_dist_sq)
            {
                best_dist_sq = dist_sq;
                out.proj_x = px;
                out.proj_y = py;

                const double path_yaw = std::atan2(seg_dy, seg_dx);
                out.heading_error = normalize_angle(path_yaw - yaw_);

                // Signed CTE: dot product of (proj - front_axle) with the vehicle's
                // left direction (-sin(yaw), cos(yaw)).
                // Positive  → path is to the left  → add positive steering correction.
                // Negative  → path is to the right → subtract.
                out.cte = (px - front_x) * (-std::sin(yaw_))
                        + (py - front_y) *   std::cos(yaw_);

                found = true;
            }
        }

        return found;
    }

    void update_progress()
    {
        const int start = std::max(0, std::min(last_progress_idx_, static_cast<int>(path_.size()) - 1));
        int best = start;
        double min_dist = std::numeric_limits<double>::max();

        for (int i = start; i < static_cast<int>(path_.size()); ++i)
        {
            const double d = distance_xy(x_, y_, path_[i].x, path_[i].y);
            if (d < min_dist) { min_dist = d; best = i; }
        }

        if (best > last_progress_idx_) last_progress_idx_ = best;
    }

    void publish_stop()
    {
        reset_speed_pid();
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.drive.speed = 0.0;
        msg.drive.steering_angle = 0.0;
        nav_pub_->publish(msg);
    }

    geometry_msgs::msg::Point make_point(double x, double y, double z = 0.0) const
    {
        geometry_msgs::msg::Point p;
        p.x = x; p.y = y; p.z = z;
        return p;
    }

    void publish_debug_markers(double front_x, double front_y,
                               double proj_x,  double proj_y,
                               double steering)
    {
        const rclcpp::Time t = now();
        visualization_msgs::msg::MarkerArray array;

        auto make_marker = [&](int id, int type) {
            visualization_msgs::msg::Marker m;
            m.header.frame_id = "odom";
            m.header.stamp = t;
            m.ns = "stanley";
            m.id = id;
            m.type = type;
            m.action = visualization_msgs::msg::Marker::ADD;
            m.pose.orientation.w = 1.0;
            m.lifetime = rclcpp::Duration::from_seconds(marker_lifetime_sec_);
            return m;
        };

        // Front axle sphere
        auto axle = make_marker(0, visualization_msgs::msg::Marker::SPHERE);
        axle.pose.position = make_point(front_x, front_y, 0.05);
        axle.scale.x = axle.scale.y = axle.scale.z = 0.12;
        axle.color.a = 1.0f; axle.color.r = 1.0f; axle.color.g = 0.2f; axle.color.b = 0.6f;
        array.markers.push_back(axle);

        // CTE line: front axle → nearest path projection
        auto cte_line = make_marker(1, visualization_msgs::msg::Marker::LINE_STRIP);
        cte_line.scale.x = 0.05;
        cte_line.color.a = 1.0f; cte_line.color.r = 1.0f; cte_line.color.g = 0.9f; cte_line.color.b = 0.1f;
        cte_line.points.push_back(make_point(front_x, front_y, 0.04));
        cte_line.points.push_back(make_point(proj_x,  proj_y,  0.04));
        array.markers.push_back(cte_line);

        // Steering direction arrow from vehicle center
        auto steer_arrow = make_marker(2, visualization_msgs::msg::Marker::ARROW);
        steer_arrow.scale.x = 0.04;
        steer_arrow.scale.y = 0.05;
        steer_arrow.scale.z = 0.07;
        steer_arrow.color.a = 1.0f; steer_arrow.color.r = 0.1f; steer_arrow.color.g = 1.0f; steer_arrow.color.b = 0.2f;
        const double steer_yaw = yaw_ + steering;
        steer_arrow.points.push_back(make_point(x_, y_, 0.06));
        steer_arrow.points.push_back(make_point(
            x_ + 0.30 * std::cos(steer_yaw),
            y_ + 0.30 * std::sin(steer_yaw), 0.06));
        array.markers.push_back(steer_arrow);

        debug_marker_pub_->publish(array);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        x_        = msg->pose.pose.position.x;
        y_        = msg->pose.pose.position.y;
        velocity_ = msg->twist.twist.linear.x;

        auto q = msg->pose.pose.orientation;
        tf2::Quaternion quat(q.x, q.y, q.z, q.w);
        double roll, pitch, yaw;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        yaw_ = yaw;
        have_odom_ = true;
    }

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_.clear();
        if (msg->poses.empty())
        {
            have_path_ = false;
            goal_reached_ = false;
            last_progress_idx_ = 0;
            reset_speed_pid();
            RCLCPP_WARN(get_logger(), "Received empty /plan.");
            return;
        }

        path_.reserve(msg->poses.size());
        for (const auto & ps : msg->poses)
            path_.push_back({ps.pose.position.x, ps.pose.position.y});

        have_path_ = true;
        goal_reached_ = false;
        last_progress_idx_ = 0;
        reset_speed_pid();

        RCLCPP_INFO(get_logger(),
            "Received /plan with %zu points. Start=(%.2f,%.2f) End=(%.2f,%.2f)",
            path_.size(),
            path_.front().x, path_.front().y,
            path_.back().x,  path_.back().y);
    }

    void timer_callback()
    {
        if (!have_odom_ || !have_path_ || path_.empty()) return;

        if (goal_reached_)
        {
            publish_stop();
            return;
        }

        const double goal_x = path_.back().x;
        const double goal_y = path_.back().y;
        const double dist_to_goal = distance_xy(x_, y_, goal_x, goal_y);

        if (dist_to_goal < goal_tolerance_)
        {
            goal_reached_ = true;
            publish_stop();
            RCLCPP_INFO(get_logger(), "Goal reached. dist=%.3f", dist_to_goal);
            return;
        }

        update_progress();

        // Stanley uses the front axle as its reference point, not the CoM.
        const double front_x = x_ + wheelbase_ * std::cos(yaw_);
        const double front_y = y_ + wheelbase_ * std::sin(yaw_);

        StanleyErrors errors;
        if (!compute_stanley_errors(front_x, front_y, errors))
            return;

        // Stanley steering law:
        //   δ = ψ_e  +  arctan( k · e_fa / (v + k_soft) )
        // where ψ_e is heading error and e_fa is signed CTE at the front axle.
        double steering = errors.heading_error
                        + std::atan2(stanley_k_ * errors.cte,
                                     std::abs(velocity_) + stanley_k_soft_);

        steering = std::clamp(steering, -max_steering_angle_, max_steering_angle_);

        // Speed profile: slow down near goal
        double target_speed = speed_;
        if (dist_to_goal < goal_slowdown_distance_)
            target_speed = std::max(min_speed_near_goal_, dist_to_goal);

        const double dt       = compute_dt_seconds();
        const double cmd_speed = compute_pid_speed_command(target_speed, dt);

        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.speed          = cmd_speed;
        drive_msg.drive.steering_angle = steering;
        nav_pub_->publish(drive_msg);

        // Tracking error: x = CTE, y = heading error
        geometry_msgs::msg::Vector3Stamped te_msg;
        te_msg.header.stamp    = now();
        te_msg.header.frame_id = "base_link";
        te_msg.vector.x = errors.cte;
        te_msg.vector.y = errors.heading_error;
        tracking_error_pub_->publish(te_msg);

        publish_debug_markers(front_x, front_y, errors.proj_x, errors.proj_y, steering);

        RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 500,
            "[Stanley] x=%.2f y=%.2f yaw=%.2f | cte=%.3f he=%.3f | steer=%.2f v_ref=%.2f v=%.2f",
            x_, y_, yaw_, errors.cte, errors.heading_error, steering, target_speed, velocity_);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<StanleyController>());
    rclcpp::shutdown();
    return 0;
}
