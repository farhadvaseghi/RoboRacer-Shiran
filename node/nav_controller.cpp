#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limits>

using namespace std::chrono_literals;

class NavController : public rclcpp::Node
{
public:
    NavController() : Node("nav_controller")
    {
        // -----------------------------
        // Tunable controller parameters
        // -----------------------------
        this->declare_parameter<double>("target_x");
        this->declare_parameter<double>("target_y");
        this->declare_parameter<double>("target_yaw", std::numeric_limits<double>::quiet_NaN());

        this->declare_parameter<double>("goal_tolerance", 0.1);
        this->declare_parameter<double>("yaw_tolerance", 0.1);

        this->declare_parameter<double>("approach_distance", 1.0);
        this->declare_parameter<double>("approach_tolerance", 0.5);

        this->declare_parameter<double>("wheelbase", 0.3302);

        this->declare_parameter<double>("max_steering_angle", 0.4);
        this->declare_parameter<double>("min_steering_angle", -0.4);

        this->declare_parameter<double>("max_speed", 7.0);
        this->declare_parameter<double>("min_speed", 0.0);

        this->declare_parameter<double>("kp_rho", 0.8);
        this->declare_parameter<double>("kp_alpha", 2.0);
        this->declare_parameter<double>("kp_beta", -0.5);

        this->declare_parameter<double>("timer_period_sec", 0.1);

        // -----------------------------
        // Read parameters into class members
        // -----------------------------
        target_x_ = this->get_parameter("target_x").as_double();
        target_y_ = this->get_parameter("target_y").as_double();
        target_yaw_ = this->get_parameter("target_yaw").as_double();

        goal_tolerance_ = this->get_parameter("goal_tolerance").as_double();
        yaw_tolerance_ = this->get_parameter("yaw_tolerance").as_double();

        approach_distance_ = this->get_parameter("approach_distance").as_double();
        approach_tolerance_ = this->get_parameter("approach_tolerance").as_double();

        wheelbase_ = this->get_parameter("wheelbase").as_double();

        max_steering_angle_ = this->get_parameter("max_steering_angle").as_double();
        min_steering_angle_ = this->get_parameter("min_steering_angle").as_double();

        max_speed_ = this->get_parameter("max_speed").as_double();
        min_speed_ = this->get_parameter("min_speed").as_double();

        kp_rho_ = this->get_parameter("kp_rho").as_double();
        kp_alpha_ = this->get_parameter("kp_alpha").as_double();
        kp_beta_ = this->get_parameter("kp_beta").as_double();

        const double timer_period_sec = this->get_parameter("timer_period_sec").as_double();

        // Publisher: navigation command that will later be selected by mux_controller
        nav_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/nav", 10);

        // Subscriber: vehicle state from simulator / estimation
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            std::bind(&NavController::odom_callback, this, std::placeholders::_1)
        );

        // Main control loop
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(timer_period_sec),
            std::bind(&NavController::timer_callback, this)
        );

        RCLCPP_INFO(this->get_logger(), "NavController initialized.");
    }

private:
    // ROS interfaces
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr nav_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Current robot state from /odom
    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    double velocity_ = 0.0;

    // Desired final pose
    double target_x_;
    double target_y_;
    double target_yaw_;

    // Final acceptance region for position and heading
    double goal_tolerance_;
    double yaw_tolerance_;

    // Intermediate approach point used to reach the final pose from a better direction
    double approach_distance_;
    double approach_tolerance_;
    bool approach_reached_ = false;

    // Vehicle and command limits
    double wheelbase_;
    double max_steering_angle_;
    double min_steering_angle_;
    double max_speed_;
    double min_speed_;

    // Pose controller gains
    double kp_rho_;
    double kp_alpha_;
    double kp_beta_;

    // Keep angles inside [-pi, pi]
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void timer_callback()
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;

        // If target_yaw is not provided by the user, use the heading toward the final target
        if (std::isnan(target_yaw_))
        {
            target_yaw_ = std::atan2(target_y_ - y_, target_x_ - x_);
        }

        // Build an approach point behind the final goal pose.
        // This helps the car approach the goal from a direction closer to target_yaw_.
        double approach_x = target_x_ - approach_distance_ * std::cos(target_yaw_);
        double approach_y = target_y_ - approach_distance_ * std::sin(target_yaw_);

        double dist_to_approach = std::sqrt(
            (approach_x - x_) * (approach_x - x_) +
            (approach_y - y_) * (approach_y - y_)
        );

        double ref_x, ref_y;

        // Two-phase logic:
        // 1) go to approach point
        // 2) once approach is reached, switch permanently to final target
        if (!approach_reached_)
        {
            ref_x = approach_x;
            ref_y = approach_y;

            if (dist_to_approach < approach_tolerance_)
            {
                approach_reached_ = true;
                RCLCPP_INFO(this->get_logger(), "Approach point reached. Switching to final target.");
            }
        }
        else
        {
            ref_x = target_x_;
            ref_y = target_y_;
        }

        // Position error relative to the current reference point
        double dx = ref_x - x_;
        double dy = ref_y - y_;

        // Pose controller variables
        double rho = std::sqrt(dx * dx + dy * dy);
        double alpha = normalize_angle(std::atan2(dy, dx) - yaw_);
        double beta  = normalize_angle(target_yaw_ - yaw_ - alpha);

        // Final yaw error with respect to desired goal orientation
        double yaw_error_final = normalize_angle(target_yaw_ - yaw_);

        // Stop only when both position and final orientation are acceptable
        if (rho < goal_tolerance_ && std::abs(yaw_error_final) < yaw_tolerance_)
        {
            msg.drive.speed = 0.0;
            msg.drive.steering_angle = 0.0;
            nav_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(),
                        "Goal pose reached. rho=%.3f yaw_err=%.3f",
                        rho, yaw_error_final);
            return;
        }

        // Linear speed is proportional to distance to the current reference
        double speed = kp_rho_ * rho;

        if (speed > max_speed_) speed = max_speed_;
        if (speed < min_speed_) speed = min_speed_;

        // Reduce speed when heading error is large to avoid aggressive overshoot
        if (std::abs(alpha) > 0.8)
            speed *= 0.3;
        else if (std::abs(alpha) > 0.4)
            speed *= 0.6;

        // Angular control law in pose coordinates
        double omega = kp_alpha_ * alpha + kp_beta_ * beta;

        // Convert desired yaw rate to Ackermann steering angle
        double steering = std::atan2(wheelbase_ * omega, speed);

        if (steering > max_steering_angle_) steering = max_steering_angle_;
        if (steering < min_steering_angle_) steering = min_steering_angle_;

        msg.drive.speed = speed;
        msg.drive.steering_angle = steering;

        nav_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                    "phase=%s rho=%.2f alpha=%.2f beta=%.2f x=%.2f y=%.2f yaw=%.2f ref_x=%.2f ref_y=%.2f target_yaw=%.2f steer=%.2f speed=%.2f",
                    approach_reached_ ? "FINAL" : "APPROACH",
                    rho, alpha, beta, x_, y_, yaw_, ref_x, ref_y, target_yaw_, steering, speed);
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
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavController>());
    rclcpp::shutdown();
    return 0;
}