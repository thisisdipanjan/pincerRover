#pragma once
#include <cstdint>

class USBSerial {
public:
    static void init();
    static void send(uint8_t type, const void* data, uint8_t len);
    static bool receive(uint8_t& type, uint8_t* payload, uint8_t& len);
};
