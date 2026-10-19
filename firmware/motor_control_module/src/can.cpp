#include "can.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "CAN";

Can::Can(gpio_num_t tx_pin, gpio_num_t rx_pin, int bitrate, int tx_queue_depth)
    : node_(nullptr), rx_queue_(nullptr)
{
    for (auto& slot : tx_pool_)
        slot.in_use = false;

    twai_onchip_node_config_t cfg = {};
    cfg.io_cfg.tx = tx_pin;
    cfg.io_cfg.rx = rx_pin;
    cfg.bit_timing.bitrate = static_cast<uint32_t>(bitrate);
    cfg.tx_queue_depth = static_cast<uint32_t>(tx_queue_depth);

    esp_err_t err = twai_new_node_onchip(&cfg, &node_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "new_node: %s", esp_err_to_name(err));
        return;
    }

    rx_queue_ = xQueueCreate(16, sizeof(CanFrame));

    twai_event_callbacks_t cbs = {};
    cbs.on_tx_done = on_tx_done;
    cbs.on_rx_done = on_rx_done;
    cbs.on_error   = on_error;
    err = twai_node_register_event_callbacks(node_, &cbs, this);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "register_cbs: %s", esp_err_to_name(err));
        return;
    }

    err = twai_node_enable(node_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "enable: %s", esp_err_to_name(err));
        return;
    }
    ESP_LOGI(TAG, "CAN started at %d bps", bitrate);
}

Can::~Can() {
    stop();
    if (node_)
        twai_node_delete(node_);
    if (rx_queue_)
        vQueueDelete(rx_queue_);
}

bool Can::on_rx_done(twai_node_handle_t handle,
                     const twai_rx_done_event_data_t*, void* ctx)
{
    auto self = static_cast<Can*>(ctx);

    twai_frame_t rx_frame = {};
    uint8_t buf[8];
    rx_frame.buffer = buf;
    rx_frame.buffer_len = sizeof(buf);

    if (ESP_OK == twai_node_receive_from_isr(handle, &rx_frame)) {
        CanFrame f;
        f.id   = rx_frame.header.id;
        f.extd = rx_frame.header.ide;
        f.rtr  = rx_frame.header.rtr;
        f.dlc  = rx_frame.buffer_len;
        memcpy(f.data, buf, rx_frame.buffer_len);

        xQueueSendFromISR(self->rx_queue_, &f, nullptr);
    }
    return false;
}

bool Can::on_tx_done(twai_node_handle_t,
                     const twai_tx_done_event_data_t*, void* ctx)
{
    auto self = static_cast<Can*>(ctx);

    for (auto& slot : self->tx_pool_) {
        if (slot.in_use) {
            slot.in_use = false;
            break;
        }
    }
    return false;
}

bool Can::on_error(twai_node_handle_t,
                   const twai_error_event_data_t* edata, void*)
{
    ESP_LOGW(TAG, "TWAI error: 0x%04X", edata->err_flags.val);
    return false;
}

bool Can::send(uint32_t id, const uint8_t* data, uint8_t dlc,
               bool extd, bool rtr, TickType_t timeout)
{
    int slot_idx = -1;
    for (int i = 0; i < TX_POOL_SIZE; i++) {
        if (!tx_pool_[i].in_use) {
            slot_idx = i;
            break;
        }
    }
    if (slot_idx < 0) return false;

    auto& slot = tx_pool_[slot_idx];
    slot.in_use = true;
    memcpy(slot.data, data, dlc);

    twai_frame_t frame = {};
    frame.header.id  = id;
    frame.header.ide = extd;
    frame.header.rtr = rtr;
    frame.buffer     = slot.data;
    frame.buffer_len = dlc;

    int timeout_ms = pdTICKS_TO_MS(timeout);
    esp_err_t err = twai_node_transmit(node_, &frame, timeout_ms);
    if (err != ESP_OK) {
        slot.in_use = false;
        ESP_LOGE(TAG, "transmit: %s", esp_err_to_name(err));
        return false;
    }
    return true;
}

bool Can::receive(CanFrame& frame, TickType_t timeout) {
    return xQueueReceive(rx_queue_, &frame, timeout) == pdTRUE;
}

void Can::stop() {
    if (node_)
        twai_node_disable(node_);
}

void Can::start() {
    if (node_)
        twai_node_enable(node_);
}   