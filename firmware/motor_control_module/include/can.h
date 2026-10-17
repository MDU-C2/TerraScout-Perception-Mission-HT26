#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/twai.h"

typedef struct can_driver_dev *can_driver_handle_t;

typedef struct
{
    gpio_num_t tx_pin;
    gpio_num_t rx_pin;

    twai_timing_config_t timing;
    twai_filter_config_t filter;

    bool enable_self_test;
    bool enable_listen_only;
} can_driver_config_t;

typedef struct
{
    uint32_t id;
    bool is_extended;
    bool is_rtr;
    bool is_error;
    uint8_t dlc;
    uint8_t data[8];
} can_frame_t;

esp_err_t can_driver_init(const can_driver_config_t *cfg, can_driver_handle_t *out);
void can_driver_deinit(can_driver_handle_t h);

esp_err_t can_driver_start(can_driver_handle_t h);
esp_err_t can_driver_stop(can_driver_handle_t h);

esp_err_t can_driver_send(can_driver_handle_t h, const can_frame_t *frame, TickType_t timeout_ticks);
esp_err_t can_driver_receive(can_driver_handle_t h, can_frame_t *frame, TickType_t timeout_ticks);

esp_err_t can_driver_set_alerts(can_driver_handle_t h, uint32_t alerts_to_enable, uint32_t *alerts_active);
