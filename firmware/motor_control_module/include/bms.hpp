#pragma once

#include <cstddef>
#include <cstdint>

#include "can.hpp"
#include "config.h"

struct BmsData {
    uint16_t manufacturer_id = 0;
    uint16_t sku_code = 0;
    uint16_t voltage_mv = 0;
    int32_t current_ma = 0;
    int16_t temperature_c = 0;
    uint16_t state_of_charge_pct = 0;
    uint16_t cycle_life = 0;
    uint16_t state_of_health_pct = 0;
    uint16_t cell_voltage_mv[12] = {};
    uint16_t standard_capacity_mah = 0;
    uint16_t remaining_capacity_mah = 0;
    uint32_t error_flags = 0;
};

class TattuBmsDecoder {
public:
    // Returns true only when a complete, CRC-valid BMS message was decoded.
    bool process(const Can::Frame &frame, BmsData &result);
    void reset() { active_ = false; payload_size_ = 0; }

private:
    uint8_t sequence_ = 0;
    uint8_t expected_toggle_ = 0;
    size_t payload_size_ = 0;
    bool active_ = false;
    int64_t started_us_ = 0;
    uint8_t payload_[config::TATTU_BMS_PAYLOAD_SIZE] = {};
};
