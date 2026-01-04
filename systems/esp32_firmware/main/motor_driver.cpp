#include "motor_driver.hpp"
#include "i2c_bus.hpp"

constexpr uint8_t MOTOR_ADDR = 0x34;
constexpr uint8_t MOTOR_SPEED_ADDR = 0x33;

void MotorDriver::setSpeed(int8_t left, int8_t right)
{
    uint8_t data[3] = {MOTOR_SPEED_ADDR,
                       static_cast<uint8_t>(left),
                       static_cast<uint8_t>(right)};

    xSemaphoreTake(I2CBus::mutex, portMAX_DELAY);
    i2c_master_write_to_device(
        I2CBus::port, MOTOR_ADDR, data, 3, pdMS_TO_TICKS(50));
    xSemaphoreGive(I2CBus::mutex);
}
