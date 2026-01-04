#include <cstring>
#include "i2c_bus.hpp"
#include "imu_mpu6886.hpp"
#include "encoder_odometry.hpp"
#include "motor_driver.hpp"
#include "servo_pca9685.hpp"
#include "usb_serial.hpp"

extern "C" void app_main(void)
{
    I2CBus::init(I2C_NUM_0, GPIO_NUM_21, GPIO_NUM_22);
    USBSerial::init();

    esp_err_t res = i2c_master_write_to_device(I2C_NUM_0, 0x68, nullptr, 0, 100 / portTICK_PERIOD_MS);
    if (res != ESP_OK) {
        printf("MPU6886 not found on I2C!\n");
    }
    MPU6886 imu;
    EncoderOdometry odom;
    MotorDriver motor;
    PCA9685 servo;

    imu.init();
    servo.init();

    while (true) {
        // ----- IMU -----
        int16_t ax, ay, az, gx, gy, gz;
        imu.readRaw(ax, ay, az, gx, gy, gz);

        struct {int16_t a[6];} imu_msg{{ax,ay,az,gx,gy,gz}};
        USBSerial::send(0x01, &imu_msg, sizeof(imu_msg));

        // ----- Encoders + Odom -----
        if (odom.readEncoders()) {
            odom.update(0.0f);
            struct {float x,y,th;} od{odom.x(), odom.y(), odom.theta()};
            USBSerial::send(0x02, &od, sizeof(od));
        }

        // ----- RX commands -----
        uint8_t type,len,payload[16];
        if (USBSerial::receive(type,payload,len)) {
            if (type == 0x10) {
                float steer;
                memcpy(&steer, payload, sizeof(float));
                servo.setSteering(steer);
            }
            if (type == 0x11) {
                motor.setSpeed((int8_t)payload[0], (int8_t)payload[1]);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}