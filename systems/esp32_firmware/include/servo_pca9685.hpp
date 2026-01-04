#pragma once
#include <cstdint>

class PCA9685 {
public:
    void init();
    void setServoAngle(uint8_t ch, float deg);
    void setSteering(float steering_rad);

private:
    void writePulse(uint8_t ch, uint16_t pulse);
    static uint16_t angleToPulse(float deg);

    static constexpr uint8_t SERVO_COUNT = 5;

    // Steering servo (ch0)
    static constexpr float CENTER = 95.0f;
    static constexpr float LEFT   = 150.0f;
    static constexpr float RIGHT  = 40.0f;
    static constexpr float MAX_RAD = 0.45f;

    static constexpr float MIN_DEG[SERVO_COUNT] = {40, 0, 0, 0, 0};
    static constexpr float MAX_DEG[SERVO_COUNT] = {150,180,180,180,180};
};
