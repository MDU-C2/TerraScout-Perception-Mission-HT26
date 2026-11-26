#include "bms_interface.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include <cstdio>

// Dedicated test entry point. Selected only by the bms-monitor environment.
// Pins match the current main.cpp; confirm they match your assembled wiring.
extern "C" void app_main(void)
{
    static Can battery_can;
    ESP_ERROR_CHECK(battery_can.begin(
        static_cast<gpio_num_t>(BMS_MONITOR_TX_PIN),
        static_cast<gpio_num_t>(BMS_MONITOR_RX_PIN), 1000000, 64));
    BmsInterface bms(battery_can);
    ESP_LOGI("BMS", "Battery monitor: 1 Mbit/s, TX=%d RX=%d, expected ID=0x01109216",
             BMS_MONITOR_TX_PIN, BMS_MONITOR_RX_PIN);
    int64_t last_report = 0;
    while (true) {
        Can::Frame frame;
        const auto rx = battery_can.receive(frame, 20);
        const auto now = esp_timer_get_time();
        if (rx == ESP_OK) {
            char bytes[25]{};
            for (unsigned i = 0; i < frame.dlc && i < 8; ++i)
                std::snprintf(bytes + i * 3, sizeof(bytes) - i * 3, "%02X ", frame.data[i]);
            ESP_LOGI("BMS", "t=%lld ID=0x%08lX %s RTR=%d DLC=%u data=%s",
                     static_cast<long long>(now), static_cast<unsigned long>(frame.id),
                     frame.extended ? "extended" : "standard", frame.rtr,
                     static_cast<unsigned>(frame.dlc), bytes);
            const auto result = bms.processFrame(frame, now);
            if (result == BmsInterface::Result::Updated) {
                const auto &s = *bms.latest();
                ESP_LOGI("BMS", "Updated CRC OK: voltage=%u mV current=%ld mA temp=%d C SOC=%u%% SOH=%d%% remaining=%u mAh faults=0x%08lX",
                         unsigned(s.pack_voltage_mv), static_cast<long>(s.current_ma),
                         int(s.temperature_c), unsigned(s.charge_pct), int(s.health_pct),
                         unsigned(s.remaining_capacity_mah), static_cast<unsigned long>(s.fault_flags));
                for (unsigned i = 0; i < s.cell_voltages_mv.size(); ++i)
                    ESP_LOGI("BMS", "Cell %u: %u mV", i + 1, unsigned(s.cell_voltages_mv[i]));
            } else if (result == BmsInterface::Result::CrcMismatch) {
                ESP_LOGW("BMS", "CrcMismatch: complete transfer discarded");
            } else if (result == BmsInterface::Result::Rejected) {
                ESP_LOGW("BMS", "Rejected: framing, sequence or timeout error");
            }
        } else if (rx != ESP_ERR_TIMEOUT) {
            ESP_LOGE("BMS", "CAN receive: %s", esp_err_to_name(rx));
        }
        if (now - last_report >= 1000000) {
            const auto state = bms.state(now);
            ESP_LOGI("BMS", "State=%s bus_off=%d dropped_rx=%lu",
                     state == BmsInterface::State::Fresh ? "Fresh" :
                     state == BmsInterface::State::Stale ? "Stale" : "Missing",
                     battery_can.isBusOff(), static_cast<unsigned long>(battery_can.droppedRx()));
            last_report = now;
        }
    }
}
