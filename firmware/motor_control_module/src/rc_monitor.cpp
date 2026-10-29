#include <cstdio>
#include <cstdlib>

#include "config.h"
#include "esp_timer.h"
#include "freertos/task.h"
#include "sbus.hpp"

// Receiver diagnostics only: no CAN initialization or motor commands.
extern "C" void app_main()
{
    std::puts("\nRADIO TEST ONLY - motor control is not running");
    std::printf("Throttle=CH%u  Steering=CH%u  Mode=CH%u\n",
                config::SBUS_THROTTLE_CHANNEL + 1,
                config::SBUS_STEERING_CHANNEL + 1,
                config::SBUS_MODE_CHANNEL + 1);
    std::puts("Mode is the switch REQUEST, not permission to drive.");
    Sbus sbus(config::SBUS_UART, config::SBUS_RX_PIN);
    SbusFrame latest{};
    int64_t last_print = 0;
    while (true) {
        SbusFrame incoming{};
        if (sbus.read(incoming, pdMS_TO_TICKS(50))) latest = incoming;
        const int64_t now = esp_timer_get_time();
        if (now - last_print < 500000) continue;
        last_print = now;
        const int64_t received = sbus.last_frame_time_us();
        if (!latest.valid || received <= 0 || now - received >= config::CONTROLLER_INPUT_TIMEOUT_US) {
            std::puts("RADIO: NO FRESH SIGNAL - check transmitter/receiver");
            continue;
        }
        const unsigned throttle = latest.channels[config::SBUS_THROTTLE_CHANNEL];
        const unsigned steering = latest.channels[config::SBUS_STEERING_CHANNEL];
        const unsigned sw = latest.channels[config::SBUS_MODE_CHANNEL];
        const char *mode = sw < 600 ? "DISABLED" : sw < 1400 ? "MANUAL" :
                           sw < 2000 ? "AUTO" : "INVALID";
        const bool neutral =
            std::abs(static_cast<int>(throttle) - config::SBUS_CHANNEL_MID) <= config::SBUS_DEADBAND &&
            std::abs(static_cast<int>(steering) - config::SBUS_CHANNEL_MID) <= config::SBUS_DEADBAND;
        std::printf("RADIO: %s | Throttle=%u Steering=%u | Mode=%s (%u) | Neutral=%s | Lost=%u\n",
                    latest.failsafe ? "FAILSAFE" : latest.frame_lost ? "FRAME LOST" : "OK",
                    throttle, steering, mode, sw, neutral ? "YES" : "NO",
                    static_cast<unsigned>(latest.frame_lost));
        std::fflush(stdout);
    }
}
