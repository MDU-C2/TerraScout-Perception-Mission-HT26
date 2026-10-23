#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/**
 * Minimal classic-CAN wrapper around the ESP-IDF esp_twai driver
 * (ESP-IDF >= 5.5, component: esp_driver_twai).
 *
 * - send() returns once the frame is queued in the driver (use flush() to
 *   wait until everything is actually on the bus).
 * - Received frames are pushed from the driver ISR into a FreeRTOS queue,
 *   and picked up with receive().
 */
class Can {
public:
    struct Frame {
        uint32_t id = 0;
        bool extended = false;  // true = 29-bit ID, false = 11-bit ID
        bool rtr = false;       // remote transmission request
        uint8_t dlc = 0;        // 0..8
        uint8_t data[8] = {};
    };

    Can() = default;
    ~Can();

    Can(const Can &) = delete;
    Can &operator=(const Can &) = delete;

    /**
     * Create and start the TWAI node.
     * @param self_test  Don't require ACKs (single-node testing, e.g. TX shorted to RX)
     */
    esp_err_t begin(gpio_num_t tx, gpio_num_t rx, uint32_t bitrate = 500000,
                    size_t rx_queue_len = 16, uint32_t tx_queue_depth = 8,
                    bool self_test = false);

    /** Stop and delete the node, free all resources. */
    esp_err_t end();

    /** Queue a frame for transmission (waits up to timeout_ms for queue space). */
    esp_err_t send(const Frame &frame, uint32_t timeout_ms = 100);
    esp_err_t send(uint32_t id, const uint8_t *data, uint8_t len,
                   bool extended = false, uint32_t timeout_ms = 100);

    /** Wait until all queued frames have been transmitted (needs another node to ACK). */
    esp_err_t flush(uint32_t timeout_ms = 100);

    /** Get the next received frame. timeout_ms = 0 polls without blocking. */
    esp_err_t receive(Frame &frame, uint32_t timeout_ms = 0);

    /** Start bus-off recovery (only meaningful if isBusOff()). */
    esp_err_t recover();

    bool isBusOff() const { return bus_off_; }
    uint32_t droppedRx() const { return rx_dropped_; }

private:
    static bool onRxDone(twai_node_handle_t node, const twai_rx_done_event_data_t *edata, void *ctx);
    static bool onStateChange(twai_node_handle_t node, const twai_state_change_event_data_t *edata, void *ctx);

    twai_node_handle_t node_ = nullptr;
    QueueHandle_t rx_queue_ = nullptr;
    SemaphoreHandle_t tx_mutex_ = nullptr;
    bool enabled_ = false;

    // The driver passes frames by pointer, so each queued frame and its data
    // buffer must stay valid until it has been transmitted. We keep a ring of
    // (tx_queue_depth + 1) slots: the driver can never hold more than
    // tx_queue_depth pending frames, so the slot being reused is always free.
    struct TxSlot {
        twai_frame_t frame;
        uint8_t buf[8];
    };
    TxSlot *slots_ = nullptr;
    size_t slot_count_ = 0;
    size_t next_slot_ = 0;  // protected by tx_mutex_

    volatile bool bus_off_ = false;
    volatile uint32_t rx_dropped_ = 0;
};