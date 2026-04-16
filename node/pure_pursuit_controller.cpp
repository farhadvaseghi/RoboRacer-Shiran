#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <vector>
#include <utility>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

class PurePursuitController : public rclcpp::Node
{
public:
    PurePursuitController() : Node("pure_pursuit_controller")
    {
        // -----------------------------
        // Declare configurable ROS parameters
        // -----------------------------
        this->declare_parameter<double>("wheelbase", 0.3302);
        this->declare_parameter<double>("lookahead_distance", 2.0);
        this->declare_parameter<double>("speed", 1.5);
        this->declare_parameter<double>("max_steering_angle", 0.4);
        this->declare_parameter<double>("goal_tolerance", 0.3);
        this->declare_parameter<double>("timer_period_sec", 0.1);

        // -----------------------------
        // Read parameter values once at startup
        // -----------------------------
        wheelbase_ = this->get_parameter("wheelbase").as_double();
        lookahead_distance_ = this->get_parameter("lookahead_distance").as_double();
        speed_ = this->get_parameter("speed").as_double();
        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();

        const double timer_period_sec =
            this->get_parameter("timer_period_sec").as_double();

        // Publisher: navigation command sent to /nav
        nav_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
            "/nav", 10
        );

        // Subscriber: current robot state from /odom
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&PurePursuitController::odom_callback, this, std::placeholders::_1)
        );

        // Subscriber: planned path from Nav2
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

        RCLCPP_INFO(this->get_logger(), "PurePursuitController initialized. Waiting for /plan ...");
    }

private:
    // -----------------------------
    // ROS interfaces
    // -----------------------------
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr nav_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // -----------------------------
    // Current robot state from /odom
    // -----------------------------
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    double velocity_ = 0.0;
    bool have_odom_ = false;

    // -----------------------------
    // Path from /plan
    // Each point is stored as (x, y)
    // -----------------------------
    std::vector<std::pair<double, double>> path_;
    bool have_path_ = false;

    // -----------------------------
    // Controller and vehicle parameters
    // -----------------------------
    double wheelbase_;
    double lookahead_distance_;
    double speed_;
    double max_steering_angle_;
    double goal_tolerance_;

    // Keep angles inside [-pi, pi]
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

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

    void path_callback(const nav_msgs::msg::Path::SharedPtr msg)
    {
        path_.clear();

        if (msg->poses.empty())
        {
            have_path_ = false;
            RCLCPP_WARN(this->get_logger(), "Received empty /plan.");
            return;
        }

        path_.reserve(msg->poses.size());

        for (const auto & pose_stamped : msg->poses)
        {
            const double px = pose_stamped.pose.position.x;
            const double py = pose_stamped.pose.position.y;
            path_.push_back({px, py});
        }

        have_path_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Received /plan with %zu points. First=(%.2f, %.2f) Last=(%.2f, %.2f)",
            path_.size(),
            path_.front().first, path_.front().second,
            path_.back().first, path_.back().second
        );
    }

    // Find the path point closest to the current robot position
    int find_nearest_point_index()
    {
        int nearest_idx = 0;
        double min_dist = std::numeric_limits<double>::max();

        for (size_t i = 0; i < path_.size(); i++)
        {
            double dx = path_[i].first - x_;
            double dy = path_[i].second - y_;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist < min_dist)
            {
                min_dist = dist;
                nearest_idx = static_cast<int>(i);
            }
        }

        return nearest_idx;
    }

    // Starting from the nearest point, find the first point far enough away
    // to be used as the lookahead target
    int find_lookahead_point_index(int nearest_idx)
    {
        int lookahead_idx = nearest_idx;

        for (size_t i = nearest_idx; i < path_.size(); i++)
        {
            double dx = path_[i].first - x_;
            double dy = path_[i].second - y_;
            double dist = std::sqrt(dx * dx + dy * dy);

            if (dist >= lookahead_distance_)
            {
                lookahead_idx = static_cast<int>(i);
                return lookahead_idx;
            }
        }

        // If no farther point exists, use the final path point
        return static_cast<int>(path_.size() - 1);
    }

    void timer_callback()
    {
        // Do nothing until odometry and path are available
        if (!have_odom_ || !have_path_ || path_.empty())
            return;

        ackermann_msgs::msg::AckermannDriveStamped msg;

        // Stop if the robot is close enough to the final path point
        double final_dx = path_.back().first - x_;
        double final_dy = path_.back().second - y_;
        double dist_to_goal = std::sqrt(final_dx * final_dx + final_dy * final_dy);

        if (dist_to_goal < goal_tolerance_)
        {
            msg.drive.speed = 0.0;
            msg.drive.steering_angle = 0.0;
            nav_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(), "Final path point reached. Stopping vehicle.");
            return;
        }

        // 1) Find the nearest path point
        int nearest_idx = find_nearest_point_index();

        // 2) Find a lookahead point ahead of the robot
        int lookahead_idx = find_lookahead_point_index(nearest_idx);

        double target_x = path_[lookahead_idx].first;
        double target_y = path_[lookahead_idx].second;

        // 3) Compute heading error to the lookahead point
        double dx = target_x - x_;
        double dy = target_y - y_;
        double alpha = normalize_angle(std::atan2(dy, dx) - yaw_);

        // 4) Pure Pursuit steering law
        double steering = std::atan2(
            2.0 * wheelbase_ * std::sin(alpha),
            lookahead_distance_
        );

        // Clamp steering to the allowed range
        if (steering > max_steering_angle_) steering = max_steering_angle_;
        if (steering < -max_steering_angle_) steering = -max_steering_angle_;

        // Constant speed for now
        msg.drive.speed = speed_;
        msg.drive.steering_angle = steering;

        nav_pub_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "nearest_idx=%d lookahead_idx=%d target=(%.2f, %.2f) x=%.2f y=%.2f yaw=%.2f alpha=%.2f steer=%.2f speed=%.2f",
            nearest_idx, lookahead_idx, target_x, target_y,
            x_, y_, yaw_, alpha, steering, speed_
        );
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PurePursuitController>());
    rclcpp::shutdown();
    return 0;
}