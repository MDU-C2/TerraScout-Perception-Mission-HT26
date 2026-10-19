#pragma once

#include "driver/gpio.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <cstdint>

struct CanFrame {
    uint32_t id;
    bool     extd;
    bool     rtr;
    uint8_t  dlc;
    uint8_t  data[8];
};

class Can {
public:
    Can(gpio_num_t tx_pin, gpio_num_t rx_pin, int bitrate = 500000,
        int tx_queue_depth = 8);
    ~Can();

    Can(Can const&) = delete;
    Can& operator=(Can const&) = delete;

    bool send(uint32_t id, const uint8_t* data, uint8_t dlc,
              bool extd = false, bool rtr = false,
              TickType_t timeout = pdMS_TO_TICKS(100));

    bool receive(CanFrame& frame, TickType_t timeout = pdMS_TO_TICKS(100));

    void stop();
    void start();

private:
    twai_node_handle_t  node_;
    QueueHandle_t       rx_queue_;

    static constexpr int TX_POOL_SIZE = 8;
    struct TxSlot {
        twai_frame_t frame;
        uint8_t      data[8];
        bool         in_use;
    };
    TxSlot tx_pool_[TX_POOL_SIZE];

    // ISR callbacks
    static bool on_rx_done(twai_node_handle_t,
                           const twai_rx_done_event_data_t*, void*);
    static bool on_tx_done(twai_node_handle_t,
                           const twai_tx_done_event_data_t*, void*);
    static bool on_error(twai_node_handle_t,
                         const twai_error_event_data_t*, void*);
};   