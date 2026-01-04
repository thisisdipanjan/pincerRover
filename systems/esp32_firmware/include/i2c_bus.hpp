#pragma once
#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

class I2CBus {
public:
    static esp_err_t init(i2c_port_t port, gpio_num_t sda, gpio_num_t scl);
    static SemaphoreHandle_t mutex;
    static i2c_port_t port;
};
