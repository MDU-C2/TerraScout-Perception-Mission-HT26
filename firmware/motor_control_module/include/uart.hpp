#pragma once

#include "driver/gpio.h"
#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstddef>
#include <cstdint>

class Uart
{
public:
    Uart(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate);
    ~Uart();

    void send(const char* data);
    int receive(uint8_t* buf, size_t len, TickType_t timeout);
private:
    uart_port_t uart_num_;
};
