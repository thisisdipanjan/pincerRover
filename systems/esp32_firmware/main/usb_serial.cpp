#include "usb_serial.hpp"
#include "driver/uart.h"
#include <cstring>

#define USB_UART UART_NUM_0
#define RX_BUF 128

static uint8_t rxbuf[RX_BUF];

void USBSerial::init()
{
    uart_driver_install(USB_UART, RX_BUF, RX_BUF, 0, nullptr, 0);
}

void USBSerial::send(uint8_t type, const void* data, uint8_t len)
{
    uint8_t frame[64];
    uint8_t idx = 0, crc = 0;

    frame[idx++] = 0xAA;
    frame[idx++] = type;
    frame[idx++] = len;
    memcpy(&frame[idx], data, len);
    idx += len;

    for (uint8_t i = 0; i < idx; i++) crc ^= frame[i];
    frame[idx++] = crc;

    uart_write_bytes(USB_UART, (char*)frame, idx);
}

bool USBSerial::receive(uint8_t& type, uint8_t* payload, uint8_t& len)
{
    int n = uart_read_bytes(USB_UART, rxbuf, RX_BUF, 10 / portTICK_PERIOD_MS);
    if (n < 5) return false;

    for (int i = 0; i < n - 4; i++) {
        if (rxbuf[i] == 0xAA) {
            type = rxbuf[i + 1];
            len  = rxbuf[i + 2];
            memcpy(payload, &rxbuf[i + 3], len);
            return true;
        }
    }
    return false;
}
