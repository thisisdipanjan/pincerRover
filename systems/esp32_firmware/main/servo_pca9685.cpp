#include "servo_pca9685.hpp"
#include "i2c_bus.hpp"
#include <algorithm>

#define SERVO_ADDR 0x40
#define MODE1 0x00
#define LED0_ON_L 0x06

void PCA9685::init()
{
    uint8_t cmd[2] = {MODE1, 0x00};
    xSemaphoreTake(I2CBus::mutex, portMAX_DELAY);
    i2c_master_write_to_device(I2CBus::port, SERVO_ADDR, cmd, 2, 50);
    xSemaphoreGive(I2CBus::mutex);
}

void PCA9685::setServoAngle(uint8_t ch, float deg)
{
    if (ch >= SERVO_COUNT) return;
    deg = std::clamp(deg, MIN_DEG[ch], MAX_DEG[ch]);
    writePulse(ch, angleToPulse(deg));
}

void PCA9685::setSteering(float rad)
{
    rad = std::clamp(rad, -MAX_RAD, MAX_RAD);
    float deg = (rad >= 0)
        ? CENTER + (LEFT - CENTER) * (rad / MAX_RAD)
        : CENTER + (CENTER - RIGHT) * (rad / MAX_RAD);

    writePulse(0, angleToPulse(std::clamp(deg, RIGHT, LEFT)));
}

void PCA9685::writePulse(uint8_t ch, uint16_t pulse)
{
    uint8_t reg = LED0_ON_L + 4 * ch;
    uint8_t d[5] = {reg,0,0,(uint8_t)(pulse&0xFF),(uint8_t)(pulse>>8)};
    xSemaphoreTake(I2CBus::mutex, portMAX_DELAY);
    i2c_master_write_to_device(I2CBus::port, SERVO_ADDR, d, 5, 50);
    xSemaphoreGive(I2CBus::mutex);
}

uint16_t PCA9685::angleToPulse(float deg)
{
    return 150 + (deg / 180.0f) * 450;
}
