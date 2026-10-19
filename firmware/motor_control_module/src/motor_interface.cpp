#include "motor_interface.hpp"

#include <cstring>

namespace cubemars {

inline constexpr float DEG_PER_POSITION_UNIT = 0.1f;
inline constexpr int   ERPM_PER_SPEED_UNIT   = 10;
inline constexpr float AMPERE_PER_CURRENT_UNIT = 0.01f;

namespace detail {

[[nodiscard]] uint16_t readBeU16(const uint8_t* data) noexcept {
    return static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8) | data[1]);
}

[[nodiscard]] uint32_t readBeU32(const uint8_t* data) noexcept {
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
            static_cast<uint32_t>(data[3]);
}

void writeBeU32(uint8_t* data, uint32_t value) noexcept {
    data[0] = static_cast<uint8_t>((value >> 24)  & 0xFF);
    data[1] = static_cast<uint8_t>((value >> 16)  & 0xFF);
    data[2] = static_cast<uint8_t>((value >> 8)   & 0xFF);
    data[3] = static_cast<uint8_t>( value          & 0xFF);
}

} // namespace detail

twai_message_t encodeRpm(uint8_t motor_id, int32_t rpm) noexcept {
    twai_message_t msg{};

    msg.identifier = (static_cast<uint32_t>(CMD_SET_RPM) << 8) | motor_id;
    msg.extd = 1;
    msg.rtr = 0;
    msg.ss = 0;
    msg.self = 0;
    msg.dlc_non_comp = 0;
    msg.data_length_code = 4;

    detail::writeBeU32(msg.data, static_cast<uint32_t>(rpm));
    return msg;
}

[[nodiscard]] uint32_t telemetryCanId(uint8_t motor_id) noexcept {
    return (static_cast<uint32_t>(CMD_STATUS) << 8) | motor_id;
}

[[nodiscard]] bool decodeTelemetry(const uint8_t* data, size_t dlc,
                                   int /*gear_ratio*/, Telemetry& out) noexcept {
    if (!data || dlc != TELEMETRY_DLC)
        return false;

    out.position_deg    = static_cast<float>(static_cast<int16_t>(detail::readBeU16(&data[0]))) * DEG_PER_POSITION_UNIT;
    out.motor_erpm      = static_cast<int32_t>(static_cast<int16_t>(detail::readBeU16(&data[2]))) * ERPM_PER_SPEED_UNIT;
    out.current_a       = static_cast<float>(static_cast<int16_t>(detail::readBeU16(&data[4]))) * AMPERE_PER_CURRENT_UNIT;
    out.temperature_c   = static_cast<int8_t>(data[6]);
    out.error_code      = data[7];
    return true;
}

[[nodiscard]] const char* describeError(uint8_t error_code) noexcept {
    switch (error_code) {
        case 0:  return "MOTOR OK";
        case 1:  return "MOTOR OVER TEMPERATURE";
        case 2:  return "MOTOR OVER CURRENT";
        case 3:  return "MOTOR OVER VOLTAGE";
        case 4:  return "MOTOR UNDER VOLTAGE";
        case 5:  return "ENCODER FAULT";
        case 6:  return "MOSFET OVER TEMPERATURE";
        case 7:  return "MOTOR STALL";
        default: return "UNKNOWN ERROR";
    }
}

} // namespace cubemars