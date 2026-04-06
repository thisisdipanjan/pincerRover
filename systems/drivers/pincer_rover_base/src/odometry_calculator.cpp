#include "pincer_rover_base/odometry_calculator.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cmath>


OdometryCalculator::OdometryCalculator(double wheel_base_m,
                                       double wheel_circumference,
                                       int    encoder_cpr,
                                       double publish_dt_s)
: meters_per_count_(wheel_circumference / static_cast<double>(encoder_cpr)),
  wheel_base_m_(wheel_base_m),
  dt_s_(publish_dt_s),
  x_(0.0), y_(0.0), theta_(0.0),
  last_left_(0), last_right_(0),
  initialized_(false)
{}

nav_msgs::msg::Odometry OdometryCalculator::update(int32_t      left_counts,
                                                    int32_t      right_counts,
                                                    rclcpp::Time stamp)
{
    nav_msgs::msg::Odometry odom;

    if (!initialized_) {
        last_left_   = left_counts;
        last_right_  = right_counts;
        initialized_ = true;
        odom.header.stamp    = stamp;
        odom.header.frame_id = "odom";
        odom.child_frame_id  = "base_link";
        return odom;
    }

    const int32_t dl = left_counts  - last_left_;
    const int32_t dr = right_counts - last_right_;
    last_left_  = left_counts;
    last_right_ = right_counts;

    const double dist_l = static_cast<double>(dl) * meters_per_count_;
    const double dist_r = static_cast<double>(dr) * meters_per_count_;

    const double dist   = (dist_l + dist_r) * 0.5;
    const double dtheta = (dist_r - dist_l) / wheel_base_m_;

    const double mid_theta = theta_ + dtheta * 0.5;
    x_     += dist * std::cos(mid_theta);
    y_     += dist * std::sin(mid_theta);
    theta_ += dtheta;

    // Normalise heading to (-π, π]
    while (theta_ >  M_PI) theta_ -= 2.0 * M_PI;
    while (theta_ <= -M_PI) theta_ += 2.0 * M_PI;

    odom.header.stamp    = stamp;
    odom.header.frame_id = "odom";
    odom.child_frame_id  = "base_link";

    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, theta_);
    odom.pose.pose.orientation = tf2::toMsg(q);

    // Covariance: diagonal, modest uncertainty for a wheel-encoder system
    odom.pose.covariance[0]  = 0.01;   // x
    odom.pose.covariance[7]  = 0.01;   // y
    odom.pose.covariance[35] = 0.05;   // yaw

    // Velocities over the publish window
    odom.twist.twist.linear.x  = dist   / dt_s_;
    odom.twist.twist.angular.z = dtheta / dt_s_;

    odom.twist.covariance[0]  = 0.01;
    odom.twist.covariance[35] = 0.05;

    return odom;
}

void OdometryCalculator::reset()
{
    x_ = y_ = theta_ = 0.0;
    last_left_ = last_right_ = 0;
    initialized_ = false;
}
