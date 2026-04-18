#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>

using namespace std::chrono_literals;

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController() : Node("pure_pursuit_controller")
    {
        // -------------------------------------------------
        // Tunable parameters
        // -------------------------------------------------
        this->declare_parameter<double>("wheelbase", 0.3302);
        this->declare_parameter<double>("lookahead_distance", 1.0);
        this->declare_parameter<double>("speed", 2.0);
        this->declare_parameter<double>("reverse_speed", 2.0);
        this->declare_parameter<double>("max_steering_angle", 0.4);
        this->declare_parameter<double>("goal_tolerance", 0.2);
        this->declare_parameter<double>("timer_period_sec", 0.05);
        this->declare_parameter<double>("min_speed_near_goal", 0.4);
        this->declare_parameter<double>("sharp_turn_threshold", 0.25);

        // Hysteresis for switching between forward and reverse
        this->declare_parameter<double>("reverse_trigger_x", -0.15);
        this->declare_parameter<double>("forward_trigger_x", 0.15);

        // -------------------------------------------------
        // Read parameters once at startup
        // -------------------------------------------------
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        speed_ = this->get_parameter("speed").as_double();
        reverse_speed_ = this->get_parameter("reverse_speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        min_speed_near_goal_ = this->get_parameter("min_speed_near_goal").as_double();
        sharp_turn_threshold_ = this->get_parameter("sharp_turn_threshold").as_double();
        reverse_trigger_x_ = this->get_parameter("reverse_trigger_x").as_double();
        forward_trigger_x_ = this->get_parameter("forward_trigger_x").as_double();

        const double timer_period_sec =
            this->get_parameter("timer_period_sec").as_double();

        // -------------------------------------------------
        // ROS interfaces
        // -------------------------------------------------

        // Publish final Ackermann command
        nav_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/nav", 10
        );

        // Current robot state from odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1)
        );

        // Global path from planner
        plan_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/plan",
            10,
            std::bind(&PurePursuitController::path_callback, this, std::placeholders::_1)
        );

        // Main control loop
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period_sec),
            std::bind(&PurePursuitController::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(),
                    "PurePursuitController initialized. Waiting for /plan ...");
    }

private:
    // Simple 2D point used for internal path storage
    struct Point2D
    {
        double x;
        double y;
    };

    // -------------------------------------------------
    // ROS interfaces
    // -------------------------------------------------
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr nav_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // -------------------------------------------------
    // Current robot state
    // -------------------------------------------------
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    double velocity_ = 0.0;
    bool have_odom_ = false;

    // -------------------------------------------------
    // Current path state
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
    double speed_;
    double reverse_speed_;
    double max_steering_angle_;
    double goal_tolerance_;
    double min_speed_near_goal_;
    double sharp_turn_threshold_;
    double reverse_trigger_x_;
    double forward_trigger_x_;

    // Normalize angle to [-pi, pi]
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // Euclidean distance in 2D
    double distance_xy(double x1, double y1, double x2, double y2) const
    {
        const double dx = x2 - x1;
        const double dy = y2 - y1;
        return std::sqrt(dx * dx + dy * dy);
    }

    // Publish zero speed and zero steering
    void publish_stop()
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;
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

        if (msg->poses.empty())
        {
            have_path_ = false;
            goal_reached_ = false;
            reverse_mode_ = false;
            last_progress_idx_ = 0;
            RCLCPP_WARN(this->get_logger(), "Received empty /plan.");
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

        RCLCPP_INFO(this->get_logger(),
                    "Received /plan with %zu points. Start=(%.2f, %.2f) End=(%.2f, %.2f)",
                    path_.size(),
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
                                        int &target_idx)
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

            if (d >= lookahead_distance_)
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
                                       int &target_idx)
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
            const double c = fx * fx + fy * fy - lookahead_distance_ * lookahead_distance_;

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
        bool ok = find_lookahead_target_waypoint(
            last_progress_idx_, target_x, target_y, target_idx
        );

        // Option B: segment-based lookahead
        // bool ok = find_lookahead_target_segment(
        //     last_progress_idx_, target_x, target_y, target_idx
        // );

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

        double alpha = 0.0;
        double cmd_speed = 0.0;

        // -------------------------------------------------
        // Forward mode
        // -------------------------------------------------
        if (!reverse_mode_)
        {
            // Target angle relative to front-driving direction
            alpha = std::atan2(y_local, x_local);
            cmd_speed = speed_;
        }
        // -------------------------------------------------
        // Reverse mode
        // -------------------------------------------------
        else
        {
            // In reverse, target is handled relative to rear-driving direction
            alpha = std::atan2(y_local, -x_local);
            cmd_speed = -reverse_speed_;
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

        // Reduce speed magnitude for sharp turns
        if (std::abs(steering) > sharp_turn_threshold_)
            cmd_speed *= 0.6;

        // Slow down near goal while keeping the sign of speed
        if (dist_to_goal < 1.0)
        {
            double limited_speed = std::min(
                std::abs(cmd_speed),
                std::max(min_speed_near_goal_, dist_to_goal)
            );

            cmd_speed = (cmd_speed >= 0.0) ? limited_speed : -limited_speed;
        }

        // Publish control command
        ackermann_msgs::msg::AckermannDriveStamped msg;
        msg.drive.speed = cmd_speed;
        msg.drive.steering_angle = steering;
        nav_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "mode=%s x=%.2f y=%.2f yaw=%.2f nearest=%d progress=%d target_idx=%d target=(%.2f, %.2f) x_local=%.2f y_local=%.2f dist_goal=%.2f alpha=%.2f steer=%.2f speed=%.2f",
                    reverse_mode_ ? "REVERSE" : "FORWARD",
                    x_, y_, yaw_,
                    nearest_idx, last_progress_idx_, target_idx,
                    target_x, target_y,
                    x_local, y_local,
                    dist_to_goal, alpha, steering, cmd_speed);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}