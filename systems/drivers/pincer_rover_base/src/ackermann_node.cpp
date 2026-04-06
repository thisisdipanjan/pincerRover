#include "pincer_rover_base/ackermann_node.hpp"

#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <i2c/smbus.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>


static bool i2c_write_block(int fd, uint8_t addr, uint8_t reg,
                             const uint8_t *data, std::size_t len)
{
    if (ioctl(fd, I2C_SLAVE, addr) < 0) return false;
    std::vector<uint8_t> buf(len + 1);
    buf[0] = reg;
    std::memcpy(buf.data() + 1, data, len);
    return ::write(fd, buf.data(), buf.size()) == static_cast<ssize_t>(buf.size());
}


static bool i2c_write_byte(int fd, uint8_t addr, uint8_t reg, uint8_t val)
{
    return i2c_write_block(fd, addr, reg, &val, 1);
}


static bool i2c_read_block(int fd, uint8_t addr, uint8_t reg,
                            uint8_t *out, std::size_t len)
{
    if (ioctl(fd, I2C_SLAVE, addr) < 0) return false;
    if (::write(fd, &reg, 1) != 1)      return false;
    return ::read(fd, out, len) == static_cast<ssize_t>(len);
}


AckermannNode::AckermannNode()
: Node("ackermann_drive_controller"),
  queue_(16),
  odom_calc_(
      /*wheel_base_m*/        0.18,
      /*wheel_circumference*/ 0.204,   // π × 0.065 m
      /*encoder_cpr*/         1320,
      /*publish_dt_s*/        0.1)     // 10 Hz
{
    i2c_fd_ = ::open("/dev/i2c-1", O_RDWR);
    if (i2c_fd_ < 0) {
        RCLCPP_FATAL(get_logger(), "Cannot open /dev/i2c-1: %s", strerror(errno));
        throw std::runtime_error("I2C open failed");
    }

    init_motors_sync();
    init_servo_sync();

    for (auto &[name, lim] : SERVO_LIMITS) {
        set_angle_sync(servo_channel(name), lim.center);
    }

    worker_ = std::thread(&AckermannNode::i2c_worker_loop, this);

    create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        [this](geometry_msgs::msg::Twist::ConstSharedPtr m){ cmd_vel_cb(m); });

    create_subscription<std_msgs::msg::Float32>(
        "/waist_servo", 10,
        [this](std_msgs::msg::Float32::ConstSharedPtr m){
            servo_cmd(WAIST_CH, m->data, "waist"); });

    create_subscription<std_msgs::msg::Float32>(
        "/shoulder_servo", 10,
        [this](std_msgs::msg::Float32::ConstSharedPtr m){
            servo_cmd(SHOULDER_CH, m->data, "shoulder"); });

    create_subscription<std_msgs::msg::Float32>(
        "/wrist_servo", 10,
        [this](std_msgs::msg::Float32::ConstSharedPtr m){
            servo_cmd(WRIST_CH, m->data, "wrist"); });

    create_subscription<std_msgs::msg::Float32>(
        "/gripper_servo", 10,
        [this](std_msgs::msg::Float32::ConstSharedPtr m){
            servo_cmd(GRIPPER_CH, m->data, "gripper"); });

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    odom_timer_ = create_wall_timer(
        std::chrono::milliseconds(100),
        [this]{ schedule_encoder_read(); });

    RCLCPP_INFO(get_logger(), "pincer_rover_base: Ackermann node ready (odom @ 10 Hz)");
}


AckermannNode::~AckermannNode()
{
    stop_.store(true);
    if (worker_.joinable()) worker_.join();
    if (i2c_fd_ >= 0) ::close(i2c_fd_);
}


void AckermannNode::i2c_worker_loop()
{
    while (!stop_.load()) {
        I2CQueue::Job job;
        if (!queue_.pop(job, stop_)) continue;
        try {
            job(i2c_fd_);
        } catch (const std::exception &e) {
            RCLCPP_ERROR(get_logger(), "I2C job threw: %s", e.what());
        }
    }
}


void AckermannNode::init_motors_sync()
{
    uint8_t type = MOTOR_TYPE_JGB37;
    i2c_write_block(i2c_fd_, MOTOR_ADDR, MOTOR_TYPE_ADDR, &type, 1);
    usleep(10'000);

    uint8_t pol = 0;
    i2c_write_block(i2c_fd_, MOTOR_ADDR, MOTOR_POLARITY_ADDR, &pol, 1);
    usleep(10'000);

    RCLCPP_INFO(get_logger(), "Motor driver initialised");
}


void AckermannNode::init_servo_sync(int freq)
{
    i2c_write_byte(i2c_fd_, SERVO_ADDR, MODE1, 0x00);
    usleep(5'000);

    const uint8_t prescale =
        static_cast<uint8_t>(25'000'000 / (4096 * freq) - 1);
    i2c_write_byte(i2c_fd_, SERVO_ADDR, MODE1,    0x10);   // sleep mode
    i2c_write_byte(i2c_fd_, SERVO_ADDR, PRESCALE, prescale);
    i2c_write_byte(i2c_fd_, SERVO_ADDR, MODE1,    0x80);   // restart
    usleep(5'000);

    RCLCPP_INFO(get_logger(), "Servo controller initialised (freq=%d Hz)", freq);
}


void AckermannNode::set_angle_sync(int channel, double angle_deg)
{
    set_angle_on_fd(i2c_fd_, channel, angle_deg);
}


void AckermannNode::set_angle_on_fd(int fd, int channel, double angle_deg)
{
    constexpr int PULSE_MIN = 150;
    constexpr int PULSE_MAX = 600;

    const double  a     = std::clamp(angle_deg, 0.0, 180.0);
    const int     pulse = static_cast<int>(
        PULSE_MIN + (a / 180.0) * (PULSE_MAX - PULSE_MIN));

    const uint8_t reg     = LED0_ON_L + static_cast<uint8_t>(4 * channel);
    const uint8_t data[4] = {
        0,
        0,
        static_cast<uint8_t>(pulse & 0xFF),
        static_cast<uint8_t>(pulse >> 8)
    };
    i2c_write_block(fd, SERVO_ADDR, reg, data, 4);
}


void AckermannNode::enqueue_drive_motors(int speed)
{
    const int clamped = std::clamp(speed, -100, 100);
    queue_.push([clamped](int fd) {
        const uint8_t val    = static_cast<uint8_t>(clamped);
        const uint8_t data[] = {val, val, val, val};
        i2c_write_block(fd, MOTOR_ADDR, MOTOR_SPEED_ADDR, data, 4);
    });
}


void AckermannNode::enqueue_set_angle(int channel, double angle_deg)
{
    queue_.push([channel, angle_deg](int fd) {
        set_angle_on_fd(fd, channel, angle_deg);
    });
}


void AckermannNode::schedule_encoder_read()
{
    // Capture the timestamp on the timer thread (accurate wall time).
    // The actual I2C read happens inside the worker — serialised with all
    // other bus traffic — but the stamp reflects when the read was scheduled.
    const rclcpp::Time stamp = now();

    queue_.push_critical([this, stamp](int fd) {
        read_encoder_and_publish(fd, stamp);
    });
}


void AckermannNode::read_encoder_and_publish(int fd, rclcpp::Time stamp)
{
    uint8_t buf[4];

    if (!i2c_read_block(fd, ENCODER_ADDR, ENC_LEFT_REG, buf, 4)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Left encoder read failed");
        return;
    }
    int32_t left_counts;
    std::memcpy(&left_counts, buf, 4);

    if (!i2c_read_block(fd, ENCODER_ADDR, ENC_RIGHT_REG, buf, 4)) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
                             "Right encoder read failed");
        return;
    }
    int32_t right_counts;
    std::memcpy(&right_counts, buf, 4);

    auto odom_msg = odom_calc_.update(left_counts, right_counts, stamp);

    // Don't publish the empty baseline message on the very first call
    if (odom_msg.header.frame_id.empty()) return;

    odom_pub_->publish(odom_msg);
}


void AckermannNode::cmd_vel_cb(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
    // Motor speed: scale linear.x → [-100, 100]
    enqueue_drive_motors(static_cast<int>(-msg->linear.x * 50.0));

    // Steering angle
    const auto &lim = SERVO_LIMITS.at("steering");
    const double az = -msg->angular.z;

    if (az != 0.0) {
        steering_angle_ += az * 10.0;
        steering_angle_  = std::clamp(steering_angle_, lim.mn, lim.mx);
    } else {
        steering_angle_ = lim.center;
    }
    enqueue_set_angle(STEERING_CH, steering_angle_);
}


void AckermannNode::servo_cmd(int ch, double angle, const std::string &name)
{
    const auto &lim = SERVO_LIMITS.at(name);
    enqueue_set_angle(ch, std::clamp(angle, lim.mn, lim.mx));
}


int AckermannNode::servo_channel(const std::string &name)
{
    if (name == "steering") return STEERING_CH;
    if (name == "waist")    return WAIST_CH;
    if (name == "shoulder") return SHOULDER_CH;
    if (name == "wrist")    return WRIST_CH;
    if (name == "gripper")  return GRIPPER_CH;
    return -1;
}
