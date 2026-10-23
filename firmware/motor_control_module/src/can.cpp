#include "can.hpp"

#include <algorithm>
#include <cstring>
#include <new>

#include "esp_attr.h"
#include "esp_log.h"

static const char *TAG = "Can";

Can::~Can()
{
    end();
}

esp_err_t Can::begin(gpio_num_t tx, gpio_num_t rx, uint32_t bitrate,
                     size_t rx_queue_len, uint32_t tx_queue_depth, bool self_test)
{
    if (node_) {
        return ESP_ERR_INVALID_STATE;
    }

    rx_queue_ = xQueueCreate(rx_queue_len, sizeof(Frame));
    tx_mutex_ = xSemaphoreCreateMutex();
    slot_count_ = tx_queue_depth + 1;
    slots_ = new (std::nothrow) TxSlot[slot_count_]();
    next_slot_ = 0;
    if (!rx_queue_ || !tx_mutex_ || !slots_) {
        end();
        return ESP_ERR_NO_MEM;
    }

    twai_onchip_node_config_t cfg = {};
    cfg.io_cfg.tx = tx;
    cfg.io_cfg.rx = rx;
    cfg.io_cfg.quanta_clk_out = GPIO_NUM_NC;     // optional pins unused
    cfg.io_cfg.bus_off_indicator = GPIO_NUM_NC;
    cfg.bit_timing.bitrate = bitrate;
    cfg.tx_queue_depth = tx_queue_depth;
    cfg.fail_retry_cnt = -1;                     // retry until success or bus-off
    cfg.flags.enable_self_test = self_test;

    esp_err_t err = twai_new_node_onchip(&cfg, &node_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip failed: %s", esp_err_to_name(err));
        node_ = nullptr;
        end();
        return err;
    }

    // Callbacks must be registered before the node is enabled.
    twai_event_callbacks_t cbs = {};
    cbs.on_rx_done = onRxDone;
    cbs.on_state_change = onStateChange;
    err = twai_node_register_event_callbacks(node_, &cbs, this);
    if (err == ESP_OK) {
        err = twai_node_enable(node_);
    }
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "TWAI start failed: %s", esp_err_to_name(err));
        end();
        return err;
    }

    enabled_ = true;
    ESP_LOGI(TAG, "Started @ %lu bit/s (TX=%d, RX=%d)", (unsigned long)bitrate, (int)tx, (int)rx);
    return ESP_OK;
}

esp_err_t Can::end()
{
    if (node_) {
        if (enabled_) {
            twai_node_disable(node_);
            enabled_ = false;
        }
        twai_node_delete(node_);
        node_ = nullptr;
    }
    if (rx_queue_) {
        vQueueDelete(rx_queue_);
        rx_queue_ = nullptr;
    }
    if (tx_mutex_) {
        vSemaphoreDelete(tx_mutex_);
        tx_mutex_ = nullptr;
    }
    delete[] slots_;  // safe: node is already deleted, driver no longer references it
    slots_ = nullptr;
    slot_count_ = 0;
    bus_off_ = false;
    return ESP_OK;
}

esp_err_t Can::send(const Frame &f, uint32_t timeout_ms)
{
    if (!node_ || !enabled_) {
        return ESP_ERR_INVALID_STATE;
    }
    if (f.dlc > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(tx_mutex_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    TxSlot &slot = slots_[next_slot_];
    next_slot_ = (next_slot_ + 1) % slot_count_;

    std::memcpy(slot.buf, f.data, f.dlc);

    slot.frame = twai_frame_t{};
    slot.frame.header.id = f.id;
    slot.frame.header.ide = f.extended;
    slot.frame.header.rtr = f.rtr;
    slot.frame.header.dlc = f.dlc;
    slot.frame.buffer = slot.buf;
    slot.frame.buffer_len = f.rtr ? 0 : f.dlc;

    esp_err_t err = twai_node_transmit(node_, &slot.frame, (int)timeout_ms);

    xSemaphoreGive(tx_mutex_);
    return err;
}

esp_err_t Can::flush(uint32_t timeout_ms)
{
    if (!node_ || !enabled_) {
        return ESP_ERR_INVALID_STATE;
    }
    return twai_node_transmit_wait_all_done(node_, (int)timeout_ms);
}

esp_err_t Can::send(uint32_t id, const uint8_t *data, uint8_t len, bool extended, uint32_t timeout_ms)
{
    if (len > 8 || (len > 0 && data == nullptr)) {
        return ESP_ERR_INVALID_ARG;
    }
    Frame f;
    f.id = id;
    f.extended = extended;
    f.dlc = len;
    if (len) {
        std::memcpy(f.data, data, len);
    }
    return send(f, timeout_ms);
}

esp_err_t Can::receive(Frame &frame, uint32_t timeout_ms)
{
    if (!rx_queue_) {
        return ESP_ERR_INVALID_STATE;
    }
    return xQueueReceive(rx_queue_, &frame, pdMS_TO_TICKS(timeout_ms)) == pdTRUE
               ? ESP_OK
               : ESP_ERR_TIMEOUT;
}

esp_err_t Can::recover()
{
    if (!node_) {
        return ESP_ERR_INVALID_STATE;
    }
    return twai_node_recover(node_);
}

// ---- ISR callbacks (run in interrupt context) ----

bool IRAM_ATTR Can::onRxDone(twai_node_handle_t node, const twai_rx_done_event_data_t *, void *ctx)
{
    auto *self = static_cast<Can *>(ctx);

    uint8_t buf[8];
    twai_frame_t rx = {};
    rx.buffer = buf;
    rx.buffer_len = sizeof(buf);

    if (twai_node_receive_from_isr(node, &rx) != ESP_OK) {
        return false;
    }

    Frame f;
    f.id = rx.header.id;
    f.extended = rx.header.ide;
    f.rtr = rx.header.rtr;
    f.dlc = rx.header.dlc > 8 ? 8 : (uint8_t)rx.header.dlc;
    if (!f.rtr) {
        for (uint8_t i = 0; i < f.dlc; i++) {
            f.data[i] = buf[i];
        }
    }

    BaseType_t woken = pdFALSE;
    if (xQueueSendFromISR(self->rx_queue_, &f, &woken) != pdTRUE) {
        self->rx_dropped_ = self->rx_dropped_ + 1;
    }
    return woken == pdTRUE;
}

bool IRAM_ATTR Can::onStateChange(twai_node_handle_t, const twai_state_change_event_data_t *edata, void *ctx)
{
    auto *self = static_cast<Can *>(ctx);
    self->bus_off_ = (edata->new_sta == TWAI_ERROR_BUS_OFF);
    return false;
}