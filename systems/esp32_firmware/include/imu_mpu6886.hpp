#pragma once
#include <cstdint>

class MPU6886 {
public:
    bool init();
    bool readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                 int16_t& gx, int16_t& gy, int16_t& gz);

private:
    static constexpr uint8_t IMU_ADDR = 0x68;
};
