#pragma once

#include <cstdint>
#include <cstddef>
#include "esp_twai.h"

namespace cubemars {

inline constexpr size_t   TELEMETRY_DLC = 8;
inline constexpr uint8_t  CMD_SET_RPM   = 0x02;
inline constexpr uint8_t  CMD_STATUS    = 0x09;

struct Telemetry {
    float   position_deg;
    int32_t motor_erpm;
    float   current_a;
    int8_t  temperature_c;
    uint8_t error_code;
};

twai_message_t encodeRpm(uint8_t motor_id, int32_t rpm) noexcept;

uint32_t telemetryCanId(uint8_t motor_id) noexcept;

bool decodeTelemetry(const uint8_t* data, size_t dlc, int gear_ratio, Telemetry& out) noexcept;

const char* describeError(uint8_t error_code) noexcept;

} // namespace cubemars