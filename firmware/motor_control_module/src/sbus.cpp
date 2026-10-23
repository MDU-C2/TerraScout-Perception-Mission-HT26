#include "sbus.hpp"

#include <algorithm>
#include <cstring>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/task.h"

static constexpr uint8_t SBUS_HEADER = 0x0F;
static constexpr int     SBUS_BAUD = 100000; // SBUS standard baud rate
static constexpr int     UART_RX_BUF = 256;

Sbus::Sbus(uart_port_t port, gpio_num_t rx_pin) : uart_num_(port)
{
    uart_config_t cfg = {};
    cfg.baud_rate = SBUS_BAUD;
    cfg.data_bits = UART_DATA_8_BITS;
    cfg.parity = UART_PARITY_EVEN;
    cfg.stop_bits = UART_STOP_BITS_2;
    cfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    cfg.source_clk = UART_SCLK_DEFAULT;

    // Init SBUS driver and configure SBUS parameters - crash on error
    ESP_ERROR_CHECK(uart_param_config(uart_num_, &cfg));
    
    ESP_ERROR_CHECK(uart_set_pin(uart_num_, UART_PIN_NO_CHANGE, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    
    // SBUS signal uses invert
    ESP_ERROR_CHECK(uart_set_line_inverse(uart_num_, UART_SIGNAL_RXD_INV));
    
    ESP_ERROR_CHECK(uart_driver_install(uart_num_, UART_RX_BUF, 0, 0, nullptr, 0));
}

Sbus::~Sbus()
{
    if (uart_is_driver_installed(uart_num_)) {
        uart_driver_delete(uart_num_);
    }
}

bool Sbus::footer_ok(uint8_t b)
{
    return b == 0x00 || (b & 0x0F) == 0x04;
}

void Sbus::push_byte(uint8_t b, SbusFrame &latest, bool &got)
{
    if (win_len_ == FRAME_LEN) {
        memmove(win_, win_ + 1, FRAME_LEN - 1);
        win_len_ = FRAME_LEN - 1;
    }
    win_[win_len_++] = b;

    while (win_len_ > 0 && win_[0] != SBUS_HEADER) {
        memmove(win_, win_ + 1, win_len_ - 1);
        win_len_--;
    }

    if (win_len_ == FRAME_LEN) {
        if (footer_ok(win_[FRAME_LEN - 1])) {
            parse_frame(win_, latest);
            last_frame_us_ = esp_timer_get_time();
            good_frames_++;
            got = true;
            win_len_ = 0;
        } else {
            bad_windows_++;
        }
    }
}

bool Sbus::read(SbusFrame &frame, TickType_t timeout)
{
    if (!uart_is_driver_installed(uart_num_)) {
        return false;
    }

    uint8_t rx[64];
    bool got = false;
    const TickType_t start = xTaskGetTickCount();

    while (true) {
        TickType_t remaining = timeout;
        if (timeout != portMAX_DELAY) {
            const TickType_t elapsed = xTaskGetTickCount() - start;
            if (elapsed >= timeout) {
                break;
            }
            remaining = timeout - elapsed;
        }

        int n = uart_read_bytes(uart_num_, rx, 1, remaining);
        if (n <= 0) {
            break;
        }
        push_byte(rx[0], frame, got);

        size_t avail = 0;
        uart_get_buffered_data_len(uart_num_, &avail);
        while (avail > 0) {
            n = uart_read_bytes(uart_num_, rx, std::min(avail, sizeof(rx)), 0);
            if (n <= 0) {
                break;
            }
            for (int i = 0; i < n; i++) {
                push_byte(rx[i], frame, got);
            }
            avail = 0;
            uart_get_buffered_data_len(uart_num_, &avail);
        }

        if (got) {
            break;
        }
    }

    return got;
}

bool Sbus::has_signal(int64_t timeout_us) const
{
    return last_frame_us_ != 0 &&
           (esp_timer_get_time() - last_frame_us_) <= timeout_us;
}

void Sbus::parse_frame(const uint8_t *d, SbusFrame &frame)
{
    uint32_t acc = 0;
    int bits = 0, ch = 0;
    for (int i = 1; i <= 22 && ch < 16; i++) {
        acc |= static_cast<uint32_t>(d[i]) << bits;
        bits += 8;
        while (bits >= 11 && ch < 16) {
            frame.channels[ch++] = acc & 0x07FF;
            acc >>= 11;
            bits -= 11;
        }
    }

    frame.ch17 = (d[23] & 0x01) != 0;
    frame.ch18 = (d[23] & 0x02) != 0;
    frame.frame_lost = (d[23] & 0x04) != 0;
    frame.failsafe = (d[23] & 0x08) != 0;
    frame.valid = true;
}