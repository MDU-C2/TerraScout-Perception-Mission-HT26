#include <cstdio>

#include "sbus.hpp"
#include "can.hpp"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

constexpr char TAG[] = "MAIN";

constexpr uart_port_t SBUS_UART = UART_NUM_1;
constexpr gpio_num_t SBUS_RX_PIN = GPIO_NUM_2;

constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_5;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_4;
constexpr uint32_t CAN_BITRATE = 500000;

void sbus_task(void *)
{
    Sbus sbus(SBUS_UART, SBUS_RX_PIN);
    SbusFrame frame{};

    while (true) {
        if (!sbus.read(frame, pdMS_TO_TICKS(100))) {
            ESP_LOGW("SBUS", "No SBUS frame received");
            continue;
        }

        if (!frame.valid) {
            continue;
        }

        ESP_LOGI("SBUS",
                 "CH1=%u CH2=%u CH3=%u CH4=%u "
                 "CH5=%u CH6=%u CH7=%u CH8=%u "
                 "lost=%d failsafe=%d",
                 frame.channels[0], frame.channels[1],
                 frame.channels[2], frame.channels[3],
                 frame.channels[4], frame.channels[5],
                 frame.channels[6], frame.channels[7],
                 frame.frame_lost, frame.failsafe);
    }
}

void can_task(void *)
{
    static Can can;
    if (can.begin(CAN_TX_PIN, CAN_RX_PIN, CAN_BITRATE) != ESP_OK) {
        ESP_LOGE("CAN", "Failed to initialize CAN");
        return;
    }

    const uint8_t payload[] = {1, 2, 3, 4};
    Can::Frame rx;
    while (true) {
        if (can.receive(rx, 1000) == ESP_OK) {
            printf("ID 0x%lx, DLC %d\n", (unsigned long)rx.id, rx.dlc);
        }
        if (can.isBusOff()) {
            can.recover();
        }
    }
}

extern "C" void app_main(void)
{
    // const BaseType_t result = xTaskCreate(sbus_task,"sbus_task",3072,nullptr,5,nullptr);

    // if (result != pdPASS) {
    //     ESP_LOGE(TAG, "Failed to create SBUS task");
    // }

    const BaseType_t can_result = xTaskCreate(can_task,"can_task",3072,nullptr,5,nullptr);

    if (can_result != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN task");
    }
}
