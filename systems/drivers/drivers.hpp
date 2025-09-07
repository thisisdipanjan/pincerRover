#ifndef DRIVERS_HPP
#define DRIVERS_HPP

#include <iostream>

class drivers{
    public:
        drivers();
        virtual bool start() = 0;
        virtual bool stop() = 0;
};

#endif 