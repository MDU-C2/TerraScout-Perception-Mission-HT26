#include "uart.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "UART";

Uart::Uart(uart_port_t port, gpio_num_t tx_pin, gpio_num_t rx_pin, int baud_rate)
    : uart_num_(port)
{
    uart_config_t cfg = {};
    cfg.baud_rate  = baud_rate;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_DISABLE;
    cfg.stop_bits  = UART_STOP_BITS_1;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(uart_num_, &cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "param_config: %s", esp_err_to_name(err));

    err = uart_set_pin(uart_num_, tx_pin, rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "set_pin: %s", esp_err_to_name(err));

    err = uart_driver_install(uart_num_, 2048, 0, 0, NULL, 0);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "driver_install: %s", esp_err_to_name(err));
}

Uart::~Uart() {
    uart_driver_delete(uart_num_);
}

void Uart::send(const char* data) {
    int len = uart_write_bytes(uart_num_, data, strlen(data));
    ESP_LOGI(TAG, "Sent %d bytes", len);
}

int Uart::receive(uint8_t* buf, size_t len, TickType_t timeout) {
    return uart_read_bytes(uart_num_, buf, len, timeout);
}