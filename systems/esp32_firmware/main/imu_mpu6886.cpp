#include "imu_mpu6886.hpp"
#include "i2c_bus.hpp"
#include "driver/i2c.h"

bool MPU6886::init()
{
    uint8_t cmd[2] = {0x6B, 0x00}; // Wake up
    xSemaphoreTake(I2CBus::mutex, portMAX_DELAY);
    esp_err_t ret = i2c_master_write_to_device(
        I2CBus::port, IMU_ADDR, cmd, 2, pdMS_TO_TICKS(50));
    xSemaphoreGive(I2CBus::mutex);
    return ret == ESP_OK;
}

bool MPU6886::readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                      int16_t& gx, int16_t& gy, int16_t& gz)
{
    uint8_t reg = 0x3B;
    uint8_t data[14];

    xSemaphoreTake(I2CBus::mutex, portMAX_DELAY);
    i2c_master_write_read_device(
        I2CBus::port, IMU_ADDR, &reg, 1, data, 14, pdMS_TO_TICKS(50));
    xSemaphoreGive(I2CBus::mutex);

    ax = (data[0] << 8) | data[1];
    ay = (data[2] << 8) | data[3];
    az = (data[4] << 8) | data[5];
    gx = (data[8] << 8) | data[9];
    gy = (data[10] << 8) | data[11];
    gz = (data[12] << 8) | data[13];
    return true;
}
