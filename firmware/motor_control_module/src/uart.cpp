#include "uart.hpp"

#include <algorithm>
#include <cstring>

#include "esp_log.h"

static const char *TAG = "UART";

Uart::Uart(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate)
    : uart_num_(port)
{
    uart_config_t cfg = {};
    cfg.baud_rate = baud_rate;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_EVEN;
    cfg.stop_bits = UART_STOP_BITS_1;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    // Init UART driver and configure parameters - crash on error
    ESP_ERROR_CHECK(uart_param_config(uart_num_, &cfg));

    ESP_ERROR_CHECK(uart_set_pin(uart_num_, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(uart_num_, 256, 256, 0, nullptr, 0));
}

Uart::~Uart()
{
    if (uart_is_driver_installed(uart_num_)) {
        uart_driver_delete(uart_num_);
    }
}

int Uart::send(const uint8_t *data, size_t len)
{
    if (!ok() || data == nullptr || len == 0) {
        return -1;
    }
    int n = uart_write_bytes(uart_num_, data, len);
    ESP_LOGD(TAG, "Sent %d bytes", n);
    return n;
}

int Uart::send(const char *str)
{
    if (str == nullptr) {
        return -1;
    }
    return send(reinterpret_cast<const uint8_t *>(str), strlen(str));
}

int Uart::receive(uint8_t *buf, size_t len, TickType_t timeout)
{
    if (!ok() || buf == nullptr || len == 0) {
        return -1;
    }

    int n = uart_read_bytes(uart_num_, buf, 1, timeout);
    if (n <= 0) {
        return n;
    }

    size_t avail = 0;
    if (len > 1 && uart_get_buffered_data_len(uart_num_, &avail) == ESP_OK && avail > 0) {
        int m = uart_read_bytes(uart_num_, buf + 1,
                                static_cast<uint32_t>(std::min(avail, len - 1)), 0);
        if (m > 0) {
            n += m;
        }
    }
    return n;
}

int Uart::receive_exact(uint8_t *buf, size_t len, TickType_t timeout)
{
    if (!ok() || buf == nullptr || len == 0) {
        return -1;
    }
    return uart_read_bytes(uart_num_, buf, static_cast<uint32_t>(len), timeout);
}

int Uart::available()
{
    if (!ok()) {
        return -1;
    }
    size_t avail = 0;
    if (uart_get_buffered_data_len(uart_num_, &avail) != ESP_OK) {
        return -1;
    }
    return static_cast<int>(avail);
}

bool Uart::flush_rx()
{
    return ok() && uart_flush_input(uart_num_) == ESP_OK;
}