#include "sensor_drivers.hpp"

sensor_drivers::sensor_drivers(){
    std::cout << "Sensor Drivers initialized" << std::endl;
}

bool sensor_drivers::init(){
    ADC.start();
    return true;
}

float sensor_drivers::getServoAngle(int servoNo){
    

}

bool sensor_drivers::stop(){
    ADC.stop();
    return true;
}