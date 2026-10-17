#include "can.h"
#include <stdlib.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "CAN";

struct can_driver_dev
{
    bool started;
};

static void frame_to_twai(const can_frame_t *src, twai_message_t *dst)
{
    memset(dst, 0, sizeof(*dst));
    dst->identifier = src->id;
    dst->data_length_code = src->dlc;
    dst->extd = src->is_extended;
    dst->rtr = src->is_rtr;
    dst->ss = 0;
    dst->self = 0;
    dst->dlc_non_comp = 0;
    dst->reserved = 0;
    if (!src->is_rtr && src->dlc > 0)
    {
        memcpy(dst->data, src->data, src->dlc > 8 ? 8 : src->dlc);
    }
}

static void twai_to_frame(const twai_message_t *src, can_frame_t *dst)
{
    memset(dst, 0, sizeof(*dst));
    dst->id = src->identifier;
    dst->is_extended = src->extd;
    dst->is_rtr = src->rtr;
    dst->is_error = src->ss; /* not perfect, but placeholder */
    dst->dlc = src->data_length_code;
    if (!src->rtr && dst->dlc > 0)
    {
        memcpy(dst->data, src->data, dst->dlc > 8 ? 8 : dst->dlc);
    }
}

esp_err_t can_driver_init(const can_driver_config_t *cfg, can_driver_handle_t *out)
{
    if (!cfg || !out)
        return ESP_ERR_INVALID_ARG;

    struct can_driver_dev *h = calloc(1, sizeof(*h));
    if (!h)
        return ESP_ERR_NO_MEM;

    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(cfg->tx_pin, cfg->rx_pin, cfg->enable_listen_only ? TWAI_MODE_LISTEN_ONLY : (cfg->enable_self_test ? TWAI_MODE_NO_ACK : TWAI_MODE_NORMAL));
    twai_timing_config_t t_config = cfg->timing;
    twai_filter_config_t f_config = cfg->filter;

    esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
    if (err != ESP_OK)
    {
        free(h);
        return err;
    }

    *out = h;
    ESP_LOGI(TAG, "CAN/TWAI initialized");
    return ESP_OK;
}

void can_driver_deinit(can_driver_handle_t h)
{
    if (!h)
        return;
    if (h->started)
    {
        twai_stop();
    }
    twai_driver_uninstall();
    free(h);
}

esp_err_t can_driver_start(can_driver_handle_t h)
{
    if (!h)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err = twai_start();
    if (err == ESP_OK)
        h->started = true;
    return err;
}

esp_err_t can_driver_stop(can_driver_handle_t h)
{
    if (!h)
        return ESP_ERR_INVALID_ARG;
    esp_err_t err = twai_stop();
    if (err == ESP_OK)
        h->started = false;
    return err;
}

esp_err_t can_driver_send(can_driver_handle_t h, const can_frame_t *frame, TickType_t timeout_ticks)
{
    if (!h || !frame)
        return ESP_ERR_INVALID_ARG;

    twai_message_t msg;
    frame_to_twai(frame, &msg);
    return twai_transmit(&msg, timeout_ticks);
}

esp_err_t can_driver_receive(can_driver_handle_t h, can_frame_t *frame, TickType_t timeout_ticks)
{
    if (!h || !frame)
        return ESP_ERR_INVALID_ARG;

    twai_message_t msg;
    esp_err_t err = twai_receive(&msg, timeout_ticks);
    if (err != ESP_OK)
        return err;

    twai_to_frame(&msg, frame);
    return ESP_OK;
}

esp_err_t can_driver_set_alerts(can_driver_handle_t h, uint32_t alerts_to_enable, uint32_t *alerts_active)
{
    if (!h)
        return ESP_ERR_INVALID_ARG;
    return twai_reconfigure_alerts(alerts_to_enable, alerts_active);
}