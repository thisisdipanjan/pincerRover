#include "encoder_odometry.hpp"
#include "i2c_bus.hpp"
#include "driver/i2c.h"
#include <cmath>

bool EncoderOdometry::readEncoders()
{
    uint8_t buf[8];

    xSemaphoreTake(I2CBus::mutex, portMAX_DELAY);
    esp_err_t ret = i2c_master_read_from_device(
        I2CBus::port, ENC_BASE, buf, 8, pdMS_TO_TICKS(50));
    xSemaphoreGive(I2CBus::mutex);

    if (ret != ESP_OK) return false;

    curr_left_  = (buf[0]<<24)|(buf[1]<<16)|(buf[2]<<8)|buf[3];
    curr_right_ = (buf[4]<<24)|(buf[5]<<16)|(buf[6]<<8)|buf[7];
    return true;
}

void EncoderOdometry::update(float steering_angle)
{
    int32_t dl = curr_left_  - prev_left_;
    int32_t dr = curr_right_ - prev_right_;

    prev_left_  = curr_left_;
    prev_right_ = curr_right_;

    float dist_l = (dl * 2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;
    float dist_r = (dr * 2.0f * M_PI * WHEEL_RADIUS) / TICKS_PER_REV;

    float dist = 0.5f * (dist_l + dist_r);
    float dtheta = dist * tanf(steering_angle) / WHEEL_BASE;

    x_ += dist * cosf(theta_);
    y_ += dist * sinf(theta_);
    theta_ += dtheta;
}
