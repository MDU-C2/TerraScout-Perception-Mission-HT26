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
    cbs.on_tx_done = onTxDone;
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
    if (f.dlc > 8 || f.id > (f.extended ? 0x1FFFFFFFU : 0x7FFU)) {
        return ESP_ERR_INVALID_ARG;
    }
    if (xSemaphoreTake(tx_mutex_, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        portENTER_CRITICAL(&slots_lock_);
        ++mutex_timeout_;
        portEXIT_CRITICAL(&slots_lock_);
        return ESP_ERR_TIMEOUT;
    }

    TxSlot *reserved = nullptr;
    portENTER_CRITICAL(&slots_lock_);
    for (size_t i = 0; i < slot_count_; ++i) {
        if (!slots_[i].in_use) {
            slots_[i].in_use = true;
            reserved = &slots_[i];
            break;
        }
    }
    portEXIT_CRITICAL(&slots_lock_);
    if (!reserved) {
        portENTER_CRITICAL(&slots_lock_);
        ++no_slot_;
        portEXIT_CRITICAL(&slots_lock_);
        xSemaphoreGive(tx_mutex_);
        return ESP_ERR_TIMEOUT;
    }
    TxSlot &slot = *reserved;

    std::memcpy(slot.buf, f.data, f.dlc);

    slot.frame = twai_frame_t{};
    slot.frame.header.id = f.id;
    slot.frame.header.ide = f.extended;
    slot.frame.header.rtr = f.rtr;
    slot.frame.header.dlc = f.dlc;
    slot.frame.buffer = slot.buf;
    slot.frame.buffer_len = f.rtr ? 0 : f.dlc;

    esp_err_t err = twai_node_transmit(node_, &slot.frame, (int)timeout_ms);
    if (err != ESP_OK) {
        portENTER_CRITICAL(&slots_lock_);
        slot.in_use = false;
        if (err == ESP_ERR_TIMEOUT) ++driver_timeout_;
        portEXIT_CRITICAL(&slots_lock_);
    } else {
        portENTER_CRITICAL(&slots_lock_);
        ++tx_queued_;
        portEXIT_CRITICAL(&slots_lock_);
    }

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

void Can::logDiagnostics()
{
    if (!node_) return;
    size_t used = 0;
    uint32_t queued, done, failed, unmatched, no_slot, mutex, driver;
    portENTER_CRITICAL(&slots_lock_);
    for (size_t i=0; i<slot_count_; ++i) used += slots_[i].in_use;
    queued=tx_queued_; done=tx_done_; failed=tx_failed_; unmatched=tx_unmatched_;
    no_slot=no_slot_; mutex=mutex_timeout_; driver=driver_timeout_;
    portEXIT_CRITICAL(&slots_lock_);
    twai_node_status_t status{};
    twai_node_record_t record{};
    const esp_err_t err = twai_node_get_info(node_, &status, &record);
    ESP_LOGE(TAG, "TX DIAG slots=%u/%u queued=%lu done=%lu failed=%lu unmatched=%lu "
        "timeouts(slot/mutex/driver)=%lu/%lu/%lu",
        static_cast<unsigned>(used), static_cast<unsigned>(slot_count_),
        static_cast<unsigned long>(queued), static_cast<unsigned long>(done),
        static_cast<unsigned long>(failed), static_cast<unsigned long>(unmatched),
        static_cast<unsigned long>(no_slot), static_cast<unsigned long>(mutex),
        static_cast<unsigned long>(driver));
    if (err == ESP_OK) {
        ESP_LOGE(TAG, "BUS DIAG state=%d TXerrors=%u RXerrors=%u queue_free=%lu bus_errors=%lu RXdropped=%lu",
            static_cast<int>(status.state), status.tx_error_count, status.rx_error_count,
            static_cast<unsigned long>(status.tx_queue_remaining),
            static_cast<unsigned long>(record.bus_err_num), static_cast<unsigned long>(rx_dropped_));
    } else ESP_LOGE(TAG, "BUS DIAG unavailable: %s", esp_err_to_name(err));
}

bool IRAM_ATTR Can::onTxDone(twai_node_handle_t,
                             const twai_tx_done_event_data_t *edata, void *ctx)
{
    auto *self = static_cast<Can *>(ctx);
    portENTER_CRITICAL_ISR(&self->slots_lock_);
    ++self->tx_done_;
    if (!edata->is_tx_success) ++self->tx_failed_;
    bool matched = false;
    for (size_t i = 0; i < self->slot_count_; ++i) {
        if (&self->slots_[i].frame == edata->done_tx_frame) {
            self->slots_[i].in_use = false;
            matched = true;
            break;
        }
    }
    if (!matched) ++self->tx_unmatched_;
    portEXIT_CRITICAL_ISR(&self->slots_lock_);
    return false;
}

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
