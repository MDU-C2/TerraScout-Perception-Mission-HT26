#include "bms.hpp"

#include <cstring>
#include "esp_timer.h"

namespace {

uint16_t read_u16(const uint8_t *data)
{
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

int16_t read_i16(const uint8_t *data)
{
    return static_cast<int16_t>(read_u16(data));
}

uint32_t read_u32(const uint8_t *data)
{
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

uint16_t crc16_ccitt(const uint8_t *data, size_t size)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < size; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021)
                                 : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

}  // namespace

bool TattuBmsDecoder::process(const Can::Frame &frame, BmsData &result)
{
    if (frame.id != config::TATTU_BMS_CAN_ID) {
        return false;
    }
    if (!frame.extended || frame.rtr || frame.dlc < 2 || frame.dlc > 8) {
        reset();
        return false;
    }
    const int64_t now = esp_timer_get_time();
    if (active_ && (now < started_us_ || now - started_us_ >= 100000)) reset();

    const uint8_t control = frame.data[frame.dlc - 1];
    const bool start = (control & 0x80) != 0;
    const bool end = (control & 0x40) != 0;
    const uint8_t toggle = (control >> 5) & 1;
    const uint8_t sequence = control & 0x1f;

    if (start) {
        if (end || toggle != 0 || sequence != 0) { reset(); return false; }
        // The BMS uses the low five bits as a frame sequence number (0..7),
        // not as a constant transfer ID.
        sequence_ = sequence;
        expected_toggle_ = toggle;
        payload_size_ = 0;
        active_ = true;
        started_us_ = now;
    }

    if (!active_) {
        return false;
    }
    if (sequence != sequence_ || toggle != expected_toggle_) {
        payload_size_ = 0;
        active_ = false;
        return false;
    }

    const size_t data_size = frame.dlc - 1;
    if ((!end && frame.dlc != 8) || (end && frame.dlc != 2)) {
        reset();
        return false;
    }
    if (payload_size_ + data_size > config::TATTU_BMS_PAYLOAD_SIZE) {
        payload_size_ = 0;
        active_ = false;
        return false;
    }
    std::memcpy(payload_ + payload_size_, frame.data, data_size);
    payload_size_ += data_size;
    sequence_ = (sequence_ + 1) & 0x1f;
    expected_toggle_ ^= 1;

    if (!end) {
        return false;
    }
    if (
        payload_size_ != config::TATTU_BMS_PAYLOAD_SIZE ||
        read_u16(payload_) !=
          crc16_ccitt(payload_ + 2, config::TATTU_BMS_PAYLOAD_SIZE - 2)) {
        payload_size_ = 0;
        active_ = false;
        return false;
    }

    result.manufacturer_id = read_u16(payload_ + 2);
    result.sku_code = read_u16(payload_ + 4);
    result.voltage_mv = read_u16(payload_ + 6);
    result.current_ma = static_cast<int32_t>(read_i16(payload_ + 8)) * 10;
    result.temperature_c = read_i16(payload_ + 10);
    result.state_of_charge_pct = read_u16(payload_ + 12);
    result.cycle_life = read_u16(payload_ + 14);
    result.state_of_health_pct = read_u16(payload_ + 16);
    for (size_t i = 0; i < 12; ++i) {
        result.cell_voltage_mv[i] = read_u16(payload_ + 18 + i * 2);
    }
    result.standard_capacity_mah = read_u16(payload_ + 42);
    result.remaining_capacity_mah = read_u16(payload_ + 44);
    result.error_flags = read_u32(payload_ + 46);
    payload_size_ = 0;
    active_ = false;
    return true;
}
