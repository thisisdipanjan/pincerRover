#pragma once

#include "pincer_rover_base/i2c_queue.hpp"
#include "pincer_rover_base/odometry_calculator.hpp"

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float32.hpp>

#include <atomic>
#include <map>
#include <string>
#include <thread>

/**
 * AckermannNode  (package: pincer_rover_base)
 *
 * ROS2 lifecycle node for Ackermann drive + arm servos + encoder odometry.
 *
 * I2C bus safety contract
 * ───────────────────────
 *  • Exactly ONE file descriptor and ONE worker thread own the bus.
 *  • Motor, servo, and encoder operations are all serialised through I2CQueue.
 *  • A 10 Hz wall-timer enqueues encoder reads as critical (non-droppable) jobs
 *    so they are never starved by high-frequency cmd_vel traffic.
 *  • All hardware init sleeps run synchronously before the worker starts.
 */
class AckermannNode : public rclcpp::Node
{
public:
    AckermannNode();
    ~AckermannNode() override;

private:
    static constexpr uint8_t MOTOR_ADDR          = 0x34;
    static constexpr uint8_t SERVO_ADDR          = 0x40;
    static constexpr uint8_t ENCODER_ADDR        = 0x12;

    static constexpr uint8_t MOTOR_TYPE_ADDR     = 0x14;
    static constexpr uint8_t MOTOR_POLARITY_ADDR = 0x15;
    static constexpr uint8_t MOTOR_SPEED_ADDR    = 0x33;
    static constexpr uint8_t MOTOR_TYPE_JGB37    = 3;

    static constexpr uint8_t MODE1               = 0x00;
    static constexpr uint8_t PRESCALE            = 0xFE;
    static constexpr uint8_t LED0_ON_L           = 0x06;

    // Encoder register map (adjust to match your board)
    static constexpr uint8_t ENC_LEFT_REG        = 0x00;
    static constexpr uint8_t ENC_RIGHT_REG       = 0x04;

    // Servo channels
    static constexpr int STEERING_CH = 0;
    static constexpr int WAIST_CH    = 7;
    static constexpr int SHOULDER_CH = 3;
    static constexpr int WRIST_CH    = 15;
    static constexpr int GRIPPER_CH  = 4;

    struct ServoLimit { double mn, mx, center; };
    const std::map<std::string, ServoLimit> SERVO_LIMITS = {
        {"steering", {40.0,  150.0, 95.0}},
        {"waist",    {0.0,   160.0, 70.0}},
        {"shoulder", {10.0,  160.0, 110.0}},
        {"wrist",    {60.0,  160.0, 160.0}},
        {"gripper",  {60.0,   70.0, 65.0}},
    };

    // ── I2C worker ────────────────────────────────────────────────────────
    void i2c_worker_loop();

    // ── Synchronous hardware init (runs before worker thread starts) ──────
    void init_motors_sync();
    void init_servo_sync(int freq = 50);
    void set_angle_sync(int channel, double angle_deg);

    // ── Async enqueue helpers ─────────────────────────────────────────────
    void enqueue_drive_motors(int speed);
    void enqueue_set_angle(int channel, double angle_deg);

    // ── Servo PWM (static — usable on any fd without capturing this) ──────
    static void set_angle_on_fd(int fd, int channel, double angle_deg);

    // ── Encoder / odometry ────────────────────────────────────────────────
    void schedule_encoder_read();
    void read_encoder_and_publish(int fd, rclcpp::Time stamp);

    // ── ROS callbacks ─────────────────────────────────────────────────────
    void cmd_vel_cb(geometry_msgs::msg::Twist::ConstSharedPtr msg);
    void servo_cmd(int ch, double angle, const std::string &name);

    // ── Utility ───────────────────────────────────────────────────────────
    static int servo_channel(const std::string &name);

    int               i2c_fd_{-1};
    I2CQueue          queue_;
    std::thread       worker_;
    std::atomic<bool> stop_{false};

    double steering_angle_{95.0};

    OdometryCalculator odom_calc_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::TimerBase::SharedPtr                          odom_timer_;
};
