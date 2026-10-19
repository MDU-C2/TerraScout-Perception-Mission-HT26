#include "sbus.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "SBUS";

static constexpr size_t SBUS_FRAME_LEN = 25;

Sbus::Sbus(uart_port_t port, gpio_num_t rx_pin)
    : uart_num_(port), buf_len_(0)
{
    uart_config_t cfg = {};
    cfg.baud_rate  = 100000;
    cfg.data_bits  = UART_DATA_8_BITS;
    cfg.parity     = UART_PARITY_EVEN;
    cfg.stop_bits  = UART_STOP_BITS_2;
    cfg.flow_ctrl  = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    esp_err_t err = uart_param_config(uart_num_, &cfg);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "param_config: %s", esp_err_to_name(err));

    err = uart_set_pin(uart_num_, UART_PIN_NO_CHANGE, rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "set_pin: %s", esp_err_to_name(err));

    err = uart_driver_install(uart_num_, 256, 0, 0, NULL, 0);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "driver_install: %s", esp_err_to_name(err));

    err = uart_set_line_inverse(uart_num_, UART_SIGNAL_RXD_INV);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "set_line_inverse: %s", esp_err_to_name(err));
}

Sbus::~Sbus() {
    uart_driver_delete(uart_num_);
}

bool Sbus::read(SbusFrame& frame, TickType_t timeout) {
    frame.valid = false;
    buf_len_ = 0;

    while (buf_len_ < SBUS_FRAME_LEN) {
        int n = uart_read_bytes(uart_num_,
                                buf_ + buf_len_,
                                SBUS_FRAME_LEN - buf_len_,
                                timeout);
        if (n <= 0)
            return false;  // timeout
        buf_len_ += n;

        if (buf_len_ == 1 && buf_[0] != 0x0F)
            buf_len_ = 0;
    }

    // Validate footer
    if (buf_[24] != 0x00) {
        buf_len_ = 0;
        return false;
    }

    parse_frame(buf_, frame);
    return true;
}

void Sbus::parse_frame(const uint8_t* d, SbusFrame& frame) {
    uint64_t bitbuf[3] = {0};

    for (int i = 0; i < 22; i++) {
        int bit_pos = i * 8;
        uint64_t byte_val = d[1 + i];
        for (int b = 0; b < 8; b++) {
            if (byte_val & (1 << (7 - b))) {
                int abs_bit = bit_pos + b;
                bitbuf[abs_bit / 64] |= (1ULL << (abs_bit % 64));
            }
        }
    }

    for (int ch = 0; ch < 16; ch++) {
        int bit_start = ch * 11;
        uint16_t val = 0;
        for (int b = 0; b < 11; b++) {
            int abs_bit = bit_start + b;
            uint64_t bit = (bitbuf[abs_bit / 64] >> (abs_bit % 64)) & 1;
            val = (val << 1) | bit;
        }
        frame.channels[ch] = val & 0x07FF;
    }

    frame.ch17       = (d[23] & 0x01) != 0;
    frame.ch18       = (d[23] & 0x02) != 0;
    frame.frame_lost = (d[23] & 0x04) != 0;
    frame.failsafe   = (d[23] & 0x08) != 0;
    frame.valid      = true;
}