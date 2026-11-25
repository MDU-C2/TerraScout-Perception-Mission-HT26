#include "bms_interface.hpp"
#include <cstring>

namespace {
uint16_t read16(const uint8_t *p) noexcept
{
    return static_cast<uint16_t>(p[0] | (uint16_t(p[1]) << 8));
}
int16_t signed16(const uint8_t *p) noexcept
{
    const int32_t v = read16(p);
    return static_cast<int16_t>(v >= 0x8000 ? v - 0x10000 : v);
}
uint16_t payloadCrc(const uint8_t *p, size_t length) noexcept
{
    uint16_t crc = 0xFFFF;
    while (length--) {
        crc ^= uint16_t(*p++) << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = static_cast<uint16_t>((crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1);
    }
    return crc;
}
}

BmsInterface::BmsInterface(Can &battery_can) noexcept
    : BmsInterface(battery_can, Config{})
{
}

BmsInterface::BmsInterface(Can &battery_can, Config config) noexcept
    : can_(battery_can), config_(config)
{
}

esp_err_t BmsInterface::receive(Can::Frame &frame, uint32_t timeout_ms)
{
    frame = {};
    Can::Frame received;
    const esp_err_t result = can_.receive(received, timeout_ms);
    if (result != ESP_OK)
        return result;

    if (received.dlc > sizeof(received.data) || received.id > 0x1FFFFFFFU)
        return ESP_ERR_INVALID_ARG;

    // DroneCAN and the legacy project's battery identifiers use extended frames.
    // Accepting a frame here does not confirm its sender, CRC, or payload layout.
    if (!received.extended || received.rtr)
        return ESP_ERR_NOT_SUPPORTED;

    frame = received;
    return ESP_OK;
}

void BmsInterface::discardTransfer() noexcept
{
    assembling_ = false;
    size_ = 0;
}

void BmsInterface::reset() noexcept
{
    discardTransfer();
    has_status_ = false;
    status_ = {};
}

BmsInterface::Result BmsInterface::processFrame(const Can::Frame &f,
                                                int64_t now_us) noexcept
{
    if (f.id != config_.can_id)
        return Result::Ignored;
    if (!f.extended || f.rtr || f.dlc < 2 || f.dlc > 8 || now_us < 0 ||
        (has_status_ && now_us < status_.received_us) ||
        config_.can_id > 0x1FFFFFFFU || config_.transfer_timeout_us <= 0 ||
        config_.stale_timeout_us <= 0 ||
        (config_.sequence != Sequence::ConstantTransferId &&
         config_.sequence != Sequence::IncrementingPerFrame)) {
        discardTransfer();
        return Result::Rejected;
    }

    const uint8_t tail = f.data[f.dlc - 1];
    const bool start = (tail & 0x80) != 0;
    const bool end = (tail & 0x40) != 0;
    const bool toggle = (tail & 0x20) != 0;
    const uint8_t id = tail & 0x1F;
    if (start && (end || toggle ||
        (config_.sequence == Sequence::IncrementingPerFrame && id != 0))) {
        discardTransfer();
        return Result::Rejected;
    }
    if (assembling_ && (now_us < last_frame_us_ ||
        now_us - started_us_ >= config_.transfer_timeout_us))
        discardTransfer();

    if (start) {
        discardTransfer();
        assembling_ = true;
        expected_toggle_ = false;
        expected_id_ = id;
        started_us_ = now_us;
    }
    if (!assembling_ || toggle != expected_toggle_ || id != expected_id_ ||
        (!end && f.dlc != 8) || size_ + f.dlc - 1 > transfer_.size()) {
        discardTransfer();
        return Result::Rejected;
    }
    std::memcpy(transfer_.data() + size_, f.data, f.dlc - 1);
    size_ += f.dlc - 1;
    last_frame_us_ = now_us;
    expected_toggle_ = !expected_toggle_;
    if (config_.sequence == Sequence::IncrementingPerFrame)
        expected_id_ = (id + 1) & 0x1F;

    if (!end) {
        if (size_ == transfer_.size()) {
            discardTransfer();
            return Result::Rejected;
        }
        return Result::Incomplete;
    }
    if (size_ != transfer_.size()) {
        discardTransfer();
        return Result::Rejected;
    }

    const auto *p = transfer_.data();
    BatteryStatus sample;
    sample.received_crc = read16(p);
    sample.payload_crc = payloadCrc(p + 2, 48);
    sample.payload_crc_matches = sample.received_crc == sample.payload_crc;
    if (!sample.payload_crc_matches) {
        discardTransfer();
        return Result::CrcMismatch;
    }
    sample.manufacturer_id = read16(p + 2);
    sample.sku_code = read16(p + 4);
    sample.pack_voltage_mv = read16(p + 6);
    sample.current_ma = int32_t(signed16(p + 8)) * 10;
    sample.temperature_c = signed16(p + 10);
    sample.charge_pct = read16(p + 12);
    sample.cycle_count = read16(p + 14);
    sample.health_pct = signed16(p + 16);
    for (size_t i = 0; i < sample.cell_voltages_mv.size(); ++i)
        sample.cell_voltages_mv[i] = read16(p + 18 + 2 * i);
    sample.design_capacity_mah = read16(p + 42);
    sample.remaining_capacity_mah = read16(p + 44);
    sample.fault_flags = uint32_t(read16(p + 46)) | (uint32_t(read16(p + 48)) << 16);
    sample.received_us = now_us;
    status_ = sample;
    has_status_ = true;
    discardTransfer();
    return Result::Updated;
}

BmsInterface::State BmsInterface::state(int64_t now_us) const noexcept
{
    if (!has_status_)
        return State::Missing;
    if (now_us < status_.received_us ||
        now_us - status_.received_us >= config_.stale_timeout_us)
        return State::Stale;
    return State::Fresh;
}

const BmsInterface::BatteryStatus *BmsInterface::latest() const noexcept
{
    return has_status_ ? &status_ : nullptr;
}
