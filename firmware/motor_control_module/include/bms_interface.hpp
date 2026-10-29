#pragma once

#include "can.hpp"
#include <array>

// Tattu-specific 12S format; not the standard DroneCAN BatteryInfo layout.
// One instance per pack, single-task access (or external synchronization).
class BmsInterface {
public:
    enum class Sequence { ConstantTransferId, IncrementingPerFrame };
    enum class Result { Ignored, Incomplete, Rejected, CrcMismatch, Updated };
    enum class State { Missing, Fresh, Stale };

    enum class Fault : uint32_t {
        LowTemperature = 1UL << 0,
        OverTemperature = 1UL << 1,
        ChargingOvercurrent = 1UL << 2,
        DischargingOvercurrent = 1UL << 3,
        PackUndervoltage = 1UL << 4,
        PackOvervoltage = 1UL << 5,
        CellImbalance = 1UL << 6,
        CellOvervoltage = 1UL << 7,
        CellUndervoltage = 1UL << 8,
        ChargingShortCircuit = 1UL << 9,
        DischargingShortCircuit = 1UL << 10,
        LowCharge = 1UL << 11,
        NonOriginalCharger = 1UL << 12,
    };

    struct Config {
        uint32_t can_id = 0x01109216;
        Sequence sequence = Sequence::IncrementingPerFrame;
        int64_t transfer_timeout_us = 100000;
        int64_t stale_timeout_us = 1000000;
    };

    struct BatteryStatus {
        uint16_t manufacturer_id = 0;
        uint16_t sku_code = 0;
        uint16_t pack_voltage_mv = 0;
        int32_t current_ma = 0; // positive charging, negative discharging
        int16_t temperature_c = 0;
        uint16_t charge_pct = 0;
        uint16_t cycle_count = 0;
        int16_t health_pct = 0;
        std::array<uint16_t, 12> cell_voltages_mv{};
        uint16_t design_capacity_mah = 0;
        uint16_t remaining_capacity_mah = 0;
        uint32_t fault_flags = 0; // specification bits 0..12, preserve reserved bits
        uint16_t received_crc = 0;
        uint16_t payload_crc = 0;
        bool payload_crc_matches = false;
        int64_t received_us = 0;

        bool hasFault(Fault fault) const noexcept
        { return (fault_flags & static_cast<uint32_t>(fault)) != 0; }
        bool hasAnyFault() const noexcept { return fault_flags != 0; }
    };

    // The caller owns and initializes the dedicated battery CAN instance.
    // It must outlive this interface. Use one consumer of its receive queue.
    explicit BmsInterface(Can &battery_can) noexcept;
    BmsInterface(Can &battery_can, Config config) noexcept;

    // Returns one raw extended data frame, not a validated battery sample.
    // ESP_ERR_NOT_SUPPORTED: a standard or remote-request frame was consumed.
    // Other errors, including timeout, are returned by the CAN driver.
    // Output is cleared on failure. Default polling does not block.
    esp_err_t receive(Can::Frame &frame, uint32_t timeout_ms = 0);

    // Feed frames from receive() or a shared-bus dispatcher. now_us must use
    // one monotonic clock. Updated requires complete framing and a matching
    // payload-only CRC (same coverage as William's decoder). This coverage must
    // still be confirmed with recorded battery traffic. Failures never replace
    // the last successful sample or refresh its timestamp.
    Result processFrame(const Can::Frame &frame, int64_t now_us) noexcept;
    State state(int64_t now_us) const noexcept;
    const BatteryStatus *latest() const noexcept;
    void reset() noexcept;

private:
    Can &can_;
    Config config_;
    std::array<uint8_t, 50> transfer_{};
    size_t size_ = 0;
    bool assembling_ = false;
    bool expected_toggle_ = false;
    uint8_t expected_id_ = 0;
    int64_t started_us_ = 0;
    int64_t last_frame_us_ = 0;
    BatteryStatus status_{};
    bool has_status_ = false;
    void discardTransfer() noexcept;
};
