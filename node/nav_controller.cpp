#include <rclcpp/rclcpp.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>

using namespace std::chrono_literals;

class NavController : public rclcpp::Node
{
public:
    NavController() : Node("nav_controller")
    {
        nav_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/nav", 10);

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom",10,std::bind(&NavController::odom_callback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            100ms,
            std::bind(&NavController::timer_callback, this)
        );
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr nav_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    double x_ = 0.0;
    double y_ = 0.0;
    double yaw_ = 0.0;
    double velocity_ = 0.0;
    
    double target_x_ = 5.0;
    double target_y_ = 8.0;
    double goal_tolerance_ = 0.2;

    double max_steering_angle = 0.4;
    double min_steering_angle = -0.4;

    double max_speed = 3.0;
    double min_speed = 0.5;
    
    double normalize_angle(double angle)
    {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    void timer_callback()
    {
        ackermann_msgs::msg::AckermannDriveStamped msg;
    
        double dx = target_x_ - x_;
        double dy = target_y_ - y_;
        double dist = std::sqrt(dx * dx + dy * dy);

        if (dist < goal_tolerance_)
        {
            msg.drive.speed = 0.0;
            msg.drive.steering_angle = 0.0;

            nav_pub_->publish(msg);

            RCLCPP_INFO(this->get_logger(),
                    "Target reached. Stopping vehicle. Distance = %.3f",
                    dist);
            return;
        }
        
        double desired_yaw = std::atan2(dy, dx);
        double yaw_error = normalize_angle(desired_yaw - yaw_);

        double kp_s = 1.0;
        double kp_v = 1.0;
        double steering = kp_s * yaw_error;
        double speed = kp_v * dist;

        if (steering > max_steering_angle) steering = 0.4;
        if (steering < min_steering_angle) steering = -0.4;

        if (speed > max_speed) speed = 3;
        if (speed < min_speed) speed = 0.5;

        msg.drive.speed = speed;
        msg.drive.steering_angle = steering;

        nav_pub_->publish(msg);

        RCLCPP_INFO(this->get_logger(),
                "x=%.2f y=%.2f target=(%.2f,%.2f) dist=%.2f yaw=%.2f steer=%.2f, speed=%.2f",
                x_, y_, target_x_, target_y_, dist, yaw_, steering, speed);
        
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

        //RCLCPP_INFO(this->get_logger(), "Robot position: x=%f y=%f yaw=%f velocity=%f", x, y, yaw, vx);
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NavController>());
    rclcpp::shutdown();
    return 0;
}