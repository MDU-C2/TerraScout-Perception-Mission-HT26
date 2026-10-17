#include "sbus.h"
#include <stdlib.h>
#include <string.h>
#include "esp_timer.h"
#include "esp_log.h"

static const char *TAG = "SBUS";

struct sbus_driver_dev
{
    uart_driver_handle_t uart;
    uint8_t buf[SBUS_FRAME_LEN];
    int idx;
};

static bool sbus_decode(const uint8_t *b, sbus_data_t *d)
{
    if (b[0] != SBUS_START_BYTE || b[24] != SBUS_END_BYTE)
        return false;

    d->channels[0] = ((b[1] | b[2] << 8) & 0x07FF);
    d->channels[1] = ((b[2] >> 3 | b[3] << 5) & 0x07FF);
    d->channels[2] = ((b[3] >> 6 | b[4] << 2 | b[5] << 10) & 0x07FF);
    d->channels[3] = ((b[5] >> 1 | b[6] << 7) & 0x07FF);
    d->channels[4] = ((b[6] >> 4 | b[7] << 4) & 0x07FF);
    d->channels[5] = ((b[7] >> 7 | b[8] << 1 | b[9] << 9) & 0x07FF);
    d->channels[6] = ((b[9] >> 2 | b[10] << 6) & 0x07FF);
    d->channels[7] = ((b[10] >> 5 | b[11] << 3) & 0x07FF);
    d->channels[8] = ((b[12] | b[13] << 8) & 0x07FF);
    d->channels[9] = ((b[13] >> 3 | b[14] << 5) & 0x07FF);
    d->channels[10] = ((b[14] >> 6 | b[15] << 2 | b[16] << 10) & 0x07FF);
    d->channels[11] = ((b[16] >> 1 | b[17] << 7) & 0x07FF);
    d->channels[12] = ((b[17] >> 4 | b[18] << 4) & 0x07FF);
    d->channels[13] = ((b[18] >> 7 | b[19] << 1 | b[20] << 9) & 0x07FF);
    d->channels[14] = ((b[20] >> 2 | b[21] << 6) & 0x07FF);
    d->channels[15] = ((b[21] >> 5 | b[22] << 3) & 0x07FF);

    uint8_t flags = b[23];
    d->ch17 = flags & 0x01;
    d->ch18 = flags & 0x02;
    d->frame_lost = flags & 0x04;
    d->failsafe = flags & 0x08;
    d->timestamp_us = esp_timer_get_time();
    return true;
}

esp_err_t sbus_driver_init(const sbus_driver_config_t *cfg, sbus_driver_handle_t *out)
{
    if (!cfg || !out)
        return ESP_ERR_INVALID_ARG;

    struct sbus_driver_dev *h = calloc(1, sizeof(*h));
    if (!h)
        return ESP_ERR_NO_MEM;

    uart_driver_config_t ucfg = cfg->uart_cfg;

    /* SBUS specifics */
    ucfg.baud_rate = 100000;
    ucfg.data_bits = UART_DATA_8_BITS;
    ucfg.parity = UART_PARITY_EVEN;
    ucfg.stop_bits = UART_STOP_BITS_2;
    ucfg.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
    ucfg.invert_rx = true; /* SBUS is inverted */
    ucfg.invert_tx = false;

    esp_err_t err = uart_driver_init(&ucfg, &h->uart);
    if (err != ESP_OK)
    {
        free(h);
        return err;
    }

    *out = h;
    ESP_LOGI(TAG, "SBUS initialized on UART%d", (int)ucfg.uart_num);
    return ESP_OK;
}

void sbus_driver_deinit(sbus_driver_handle_t h)
{
    if (!h)
        return;
    uart_driver_deinit(h->uart);
    free(h);
}

esp_err_t sbus_driver_read(sbus_driver_handle_t h, sbus_data_t *out, uint32_t timeout_ms)
{
    if (!h || !out)
        return ESP_ERR_INVALID_ARG;

    uint8_t tmp[64];
    size_t n = 0;
    bool got_frame = false;

    esp_err_t err = uart_driver_read(h->uart, tmp, sizeof(tmp), &n, pdMS_TO_TICKS(timeout_ms));
    if (err != ESP_OK)
        return err;
    if (n == 0)
        return ESP_ERR_TIMEOUT;

    for (size_t k = 0; k < n; k++)
    {
        uint8_t byte = tmp[k];

        if (h->idx == 0 && byte != SBUS_START_BYTE)
            continue;
        h->buf[h->idx++] = byte;

        if (h->idx == SBUS_FRAME_LEN)
        {
            if (sbus_decode(h->buf, out))
            {
                got_frame = true;
                h->idx = 0;
            }
            else
            {
                int i;
                for (i = 1; i < SBUS_FRAME_LEN; i++)
                {
                    if (h->buf[i] == SBUS_START_BYTE)
                        break;
                }
                if (i < SBUS_FRAME_LEN)
                {
                    memmove(h->buf, &h->buf[i], SBUS_FRAME_LEN - i);
                    h->idx = SBUS_FRAME_LEN - i;
                }
                else
                {
                    h->idx = 0;
                }
            }
        }
    }
    return got_frame ? ESP_OK : ESP_ERR_NOT_FOUND;
}