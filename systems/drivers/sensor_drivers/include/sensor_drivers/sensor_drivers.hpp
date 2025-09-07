#ifndef SENSOR_DRIVERS_HPP
#define SENSOR_DRIVERS_HPP

#include "drivers.hpp"
#include <ads1115rpi.h>

class sensor_drivers : public drivers {
    public:
        sensor_drivers();
        float getServoAngle(int servoNo);
        bool init();
        bool stop()override;
    private:
        ADS1115rpi ADC;
};

#endif