#pragma once

#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <cstdint>

/**
 * OdometryCalculator
 *
 * Pure odometry arithmetic — no ROS node, no I2C, no threads.
 * Takes raw encoder counts, integrates the differential-drive kinematic
 * model, and produces a fully-populated nav_msgs::msg::Odometry message.
 *
 * All methods are called exclusively from the I2C worker thread, so no
 * internal locking is needed.
 */
class OdometryCalculator
{
public:
    /**
     * @param wheel_base_m        Distance between left and right wheels (m)
     * @param wheel_circumference Wheel circumference (m) = π × diameter
     * @param encoder_cpr         Encoder counts per wheel revolution
     * @param publish_dt_s        Expected period between calls to update() (s)
     */
    OdometryCalculator(double wheel_base_m,
                       double wheel_circumference,
                       int    encoder_cpr,
                       double publish_dt_s);

    /**
     * Feed new raw encoder counts and get an odometry message back.
     *
     * @param left_counts   Cumulative left  encoder count (signed)
     * @param right_counts  Cumulative right encoder count (signed)
     * @param stamp         Timestamp to embed in the message header
     * @return              Populated Odometry message
     */
    nav_msgs::msg::Odometry update(int32_t          left_counts,
                                   int32_t          right_counts,
                                   rclcpp::Time     stamp);

    /// Reset pose and encoder baseline to zero.
    void reset();
    double x()     const { return x_; }
    double y()     const { return y_; }
    double theta() const { return theta_; }

private:
    // Config
    const double meters_per_count_;
    const double wheel_base_m_;
    const double dt_s_;

    // State
    double  x_;
    double  y_;
    double  theta_;
    int32_t last_left_;
    int32_t last_right_;
    bool    initialized_;
};
