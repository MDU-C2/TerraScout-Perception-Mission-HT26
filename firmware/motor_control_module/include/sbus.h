#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "uart.h"

#define SBUS_NUM_CHANNELS 16
#define SBUS_FRAME_LEN 25
#define SBUS_START_BYTE 0x0F
#define SBUS_END_BYTE 0x00

typedef struct
{
    uint16_t channels[SBUS_NUM_CHANNELS];
    bool ch17;
    bool ch18;
    bool frame_lost;
    bool failsafe;
    int64_t timestamp_us;
} sbus_data_t;

typedef struct sbus_driver_dev *sbus_driver_handle_t;

typedef struct
{
    uart_driver_config_t uart_cfg;
} sbus_driver_config_t;

esp_err_t sbus_driver_init(const sbus_driver_config_t *cfg, sbus_driver_handle_t *out);
void sbus_driver_deinit(sbus_driver_handle_t h);

esp_err_t sbus_driver_read(sbus_driver_handle_t h, sbus_data_t *out, uint32_t timeout_ms);

static inline uint16_t sbus_to_us(uint16_t raw)
{
    int32_t v = ((int32_t)raw - 172) * 1000 / (1811 - 172) + 1000;
    if (v < 1000)
        v = 1000;
    if (v > 2000)
        v = 2000;
    return (uint16_t)v;
}
