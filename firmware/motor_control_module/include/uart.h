#pragma once

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_err.h"

typedef struct uart_driver_dev *uart_driver_handle_t;

typedef struct
{
    uart_port_t uart_num;
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;
    int baud_rate;
    uart_word_length_t data_bits;
    uart_parity_t parity;
    uart_stop_bits_t stop_bits;
    uart_hw_flowcontrol_t flow_ctrl;
    int rx_buffer_size;
    int tx_buffer_size;
    bool invert_rx;
    bool invert_tx;
} uart_driver_config_t;

esp_err_t uart_driver_init(const uart_driver_config_t *cfg, uart_driver_handle_t *out);
void uart_driver_deinit(uart_driver_handle_t h);

esp_err_t uart_driver_write(uart_driver_handle_t h, const uint8_t *data, size_t len, TickType_t timeout_ticks);
esp_err_t uart_driver_read(uart_driver_handle_t h, uint8_t *data, size_t len, size_t *out_len, TickType_t timeout_ticks);
esp_err_t uart_driver_flush(uart_driver_handle_t h);
esp_err_t uart_driver_set_baudrate(uart_driver_handle_t h, int baud_rate);