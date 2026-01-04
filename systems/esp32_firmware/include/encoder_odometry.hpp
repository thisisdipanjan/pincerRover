#pragma once
#include <cstdint>

class EncoderOdometry {
public:
    bool readEncoders();
    void update(float steering_angle);

    float x() const { return x_; }
    float y() const { return y_; }
    float theta() const { return theta_; }

private:
    int32_t prev_left_{0}, prev_right_{0};
    int32_t curr_left_{0}, curr_right_{0};

    float x_{0.0f};
    float y_{0.0f};
    float theta_{0.0f};

    static constexpr uint8_t ENC_BASE = 0x3C;
    static constexpr float WHEEL_RADIUS = 0.034f;
    static constexpr int TICKS_PER_REV = 1316;
    static constexpr float WHEEL_BASE = 0.175f;
};
