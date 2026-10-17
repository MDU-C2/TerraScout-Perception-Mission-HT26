#include "uart.h"
#include <stdlib.h>
#include "esp_log.h"

static const char *TAG = "UART";

struct uart_driver_dev
{
    uart_port_t uart_num;
};

esp_err_t uart_driver_init(const uart_driver_config_t *cfg, uart_driver_handle_t *out)
{
    if (!cfg || !out)
        return ESP_ERR_INVALID_ARG;

    struct uart_driver_dev *h = calloc(1, sizeof(*h));
    if (!h)
        return ESP_ERR_NO_MEM;

    h->uart_num = cfg->uart_num;

    uart_config_t uart_cfg = {
        .baud_rate = cfg->baud_rate,
        .data_bits = cfg->data_bits,
        .parity = cfg->parity,
        .stop_bits = cfg->stop_bits,
        .flow_ctrl = cfg->flow_ctrl,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t err = uart_param_config(cfg->uart_num, &uart_cfg);
    if (err != ESP_OK)
        goto fail;

    err = uart_set_pin(cfg->uart_num, cfg->tx_pin, cfg->rx_pin,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK)
        goto fail;

    int rx_size = cfg->rx_buffer_size > 0 ? cfg->rx_buffer_size : 256;
    int tx_size = cfg->tx_buffer_size > 0 ? cfg->tx_buffer_size : 0;

    err = uart_driver_install(cfg->uart_num, rx_size, tx_size, 0, NULL, 0);
    if (err != ESP_OK)
        goto fail;

    uint32_t inv = 0;
    if (cfg->invert_rx)
        inv |= UART_SIGNAL_RXD_INV;
    if (cfg->invert_tx)
        inv |= UART_SIGNAL_TXD_INV;
    if (inv)
    {
        err = uart_set_line_inverse(cfg->uart_num, inv);
        if (err != ESP_OK)
            goto fail_drv;
    }

    *out = h;
    ESP_LOGI(TAG, "UART%d initialized", (int)cfg->uart_num);
    return ESP_OK;

fail_drv:
    uart_driver_delete(cfg->uart_num);
fail:
    free(h);
    return err;
}

void uart_driver_deinit(uart_driver_handle_t h)
{
    if (!h)
        return;
    uart_driver_delete(h->uart_num);
    free(h);
}

esp_err_t uart_driver_write(uart_driver_handle_t h, const uint8_t *data, size_t len, TickType_t timeout_ticks)
{
    if (!h || (!data && len > 0))
        return ESP_ERR_INVALID_ARG;
    int written = uart_write_bytes(h->uart_num, (const char *)data, len);
    if (written < 0)
        return ESP_FAIL;
    return uart_wait_tx_done(h->uart_num, timeout_ticks);
}

esp_err_t uart_driver_read(uart_driver_handle_t h, uint8_t *data, size_t len, size_t *out_len, TickType_t timeout_ticks)
{
    if (!h || (!data && len > 0) || !out_len)
        return ESP_ERR_INVALID_ARG;
    int r = uart_read_bytes(h->uart_num, data, len, timeout_ticks);
    if (r < 0)
        return ESP_FAIL;
    *out_len = (size_t)r;
    return ESP_OK;
}

esp_err_t uart_driver_flush(uart_driver_handle_t h)
{
    if (!h)
        return ESP_ERR_INVALID_ARG;
    return uart_flush(h->uart_num);
}

esp_err_t uart_driver_set_baudrate(uart_driver_handle_t h, int baud_rate)
{
    if (!h || baud_rate <= 0)
        return ESP_ERR_INVALID_ARG;
    return uart_set_baudrate(h->uart_num, baud_rate);
}