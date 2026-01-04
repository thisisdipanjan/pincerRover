#include "i2c_bus.hpp"

SemaphoreHandle_t I2CBus::mutex = nullptr;
i2c_port_t I2CBus::port = I2C_NUM_0;

esp_err_t I2CBus::init(i2c_port_t p, gpio_num_t sda, gpio_num_t scl)
{
    port = p;
    mutex = xSemaphoreCreateMutex();

    i2c_config_t conf{};
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = sda;
    conf.scl_io_num = scl;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 400000;

    ESP_ERROR_CHECK(i2c_param_config(port, &conf));
    return i2c_driver_install(port, conf.mode, 0, 0, 0);
}
