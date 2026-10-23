#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"

class Uart {
public:
    Uart(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate);
    ~Uart();

    Uart(const Uart &) = delete;
    Uart &operator=(const Uart &) = delete;

    bool ok() const { return uart_is_driver_installed(uart_num_); }

    int send(const uint8_t *data, size_t len);

    int send(const char *str);

    int receive(uint8_t *buf, size_t len, TickType_t timeout);

    int receive_exact(uint8_t *buf, size_t len, TickType_t timeout);

    int available();

    bool flush_rx();

private:
    uart_port_t uart_num_;
};