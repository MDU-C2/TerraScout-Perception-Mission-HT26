#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "config.h"

struct SbusFrame {
    uint16_t channels[16] = {};
    bool ch17 = false;
    bool ch18 = false;
    bool frame_lost = false;
    bool failsafe = false;
    bool valid = false;
};

class Sbus {
public:
    Sbus(uart_port_t port, gpio_num_t rx_pin);
    ~Sbus();

    Sbus(const Sbus &) = delete;
    Sbus &operator=(const Sbus &) = delete;

    bool read(SbusFrame &frame, TickType_t timeout);

    int64_t last_frame_time_us() const { return last_frame_us_; }

    bool has_signal(int64_t timeout_us = 100000) const;

    uint32_t good_frames() const { return good_frames_; }
    uint32_t bad_windows() const { return bad_windows_; }

private:
    void push_byte(uint8_t b, SbusFrame &latest, bool &got);
    static bool footer_ok(uint8_t b);
    static void parse_frame(const uint8_t *d, SbusFrame &frame);

    uart_port_t uart_num_;

    uint8_t win_[config::SBUS_FRAME_LEN] = {};
    size_t  win_len_ = 0;

    int64_t  last_frame_us_ = 0;
    uint32_t good_frames_ = 0;
    uint32_t bad_windows_ = 0;
};