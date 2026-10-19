#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstdint>

struct SbusFrame {
    uint16_t channels[16];
    bool     frame_lost;
    bool     failsafe;
    bool     ch17;           
    bool     ch18;
    bool     valid;
};

class Sbus {
public:
    Sbus(uart_port_t port, gpio_num_t rx_pin);
    ~Sbus();

    bool read(SbusFrame& frame, TickType_t timeout);

private:
    uart_port_t uart_num_;
    uint8_t     buf_[25];
    size_t      buf_len_;

    void parse_frame(const uint8_t* data, SbusFrame& frame);
};

