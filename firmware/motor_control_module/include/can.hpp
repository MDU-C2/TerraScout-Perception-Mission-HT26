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
    // Task-context snapshot; call at a low rate while diagnosing TX failures.
    void logDiagnostics();

    bool isBusOff() const { return bus_off_; }
    uint32_t droppedRx() const { return rx_dropped_; }

private:
    static bool onRxDone(twai_node_handle_t node, const twai_rx_done_event_data_t *edata, void *ctx);
    static bool onStateChange(twai_node_handle_t node, const twai_state_change_event_data_t *edata, void *ctx);
    static bool onTxDone(twai_node_handle_t node, const twai_tx_done_event_data_t *edata, void *ctx);

    twai_node_handle_t node_ = nullptr;
    QueueHandle_t rx_queue_ = nullptr;
    SemaphoreHandle_t tx_mutex_ = nullptr;
    bool enabled_ = false;

    // Reserve each slot until its exact frame completes. Never reuse slots
    // based on queue depth or failed enqueue attempts.
    struct TxSlot {
        twai_frame_t frame;
        uint8_t buf[8];
        bool in_use;
    };
    TxSlot *slots_ = nullptr;
    size_t slot_count_ = 0;
    portMUX_TYPE slots_lock_ = portMUX_INITIALIZER_UNLOCKED;
    uint32_t tx_queued_ = 0;
    uint32_t tx_done_ = 0;
    uint32_t tx_failed_ = 0;
    uint32_t tx_unmatched_ = 0;
    uint32_t no_slot_ = 0;
    uint32_t mutex_timeout_ = 0;
    uint32_t driver_timeout_ = 0;

    volatile bool bus_off_ = false;
    volatile uint32_t rx_dropped_ = 0;
};
