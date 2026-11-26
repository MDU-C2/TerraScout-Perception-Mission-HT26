# Battery CAN integration

## Run the dedicated USB monitor

From this firmware directory, use PlatformIO's `bms-monitor` environment:

```sh
~/.platformio/penv/bin/pio device list
~/.platformio/penv/bin/pio run -e bms-monitor -t upload --upload-port YOUR_PORT
~/.platformio/penv/bin/pio device monitor --port YOUR_PORT --baud 115200
```

Replace YOUR_PORT with the attached ESP32 port (for example a `/dev/cu.*` path
on macOS). Exit the monitor with Ctrl+C before uploading again.
This replaces the flashed application with the battery test; normal vehicle
control is not running during this test.
The test uses 1 Mbit/s, ID `0x01109216`, TX GPIO5 and RX GPIO4, matching the
current firmware's CAN pin assignments. Confirm the actual working wiring;
edit the `BMS_MONITOR_*_PIN` flags in platformio.ini if necessary.
It receives/logs frames and decodes status; it never calls CAN send(). Normal
CAN reception can still acknowledge traffic; this is not listen-only mode.

Output includes raw timestamped frames, Updated/CRC OK, CrcMismatch or Rejected,
cell voltages, and Missing/Fresh/Stale once per second. Save the raw frames if
CRC or sequence checks fail. A CRC match checks the implemented coverage only;
compare readings with the battery app. Full ESP32 build remains to be verified.

`bms_interface.hpp/.cpp` is the single firmware BMS decoder. The duplicate
`bms.hpp/.cpp` implementation has been removed; both the battery task and
standalone monitor use `BmsInterface`.

Reuse `can.hpp` and `can.cpp`; do not copy the driver for the battery.
`BmsInterface` references a caller-owned `Can` instance, exposes raw frames,
and assembles the documented Tattu 12S format (48 payload bytes plus 2 CRC bytes).
It decodes measurements but does not implement battery safety policy or standard
DroneCAN BatteryInfo. Hardware compatibility remains unverified.

## Decoding and verification

Call `processFrame(frame, now_us)` for each received frame, using one monotonic
microsecond clock. `latest()` retains the last completed sample even if a later
transfer fails; failures never refresh its timestamp. Check `state(now_us)`.
One instance handles one configured CAN ID. Access from one task or protect
all operations, including the returned status pointer, externally.

The convenience constructor uses observed ID `0x01109216`, incrementing frame sequence,
100 ms total assembly timeout and 1 s stale timeout. These are provisional
software settings, not verified hardware configuration. Use `Config` to change
them. `Sequence::ConstantTransferId` is available for that distinct convention;
it is not detected automatically. Incrementing mode requires start sequence 0.
Both modes require a
zero initial toggle, alternating toggle, start/end markers and exactly 50 bytes.

Completed samples return `Updated` only after CRC matches. A mismatch returns
`CrcMismatch` without updating the sample. CRC-16 uses initial value 0xFFFF,
polynomial 0x1021 and the 48 application bytes, excluding prefix and tails,
matching the implementation shown in William's decoder photos. Real recorded
traffic must still confirm this coverage; synthetic tests do not establish it.
`Fresh` means a recently accepted packet, not a healthy battery or permission
to drive. Measurements and all fault bits are preserved for the state machine.
`hasFault(Fault::CellUndervoltage)` and `hasAnyFault()` inspect the flags.

The layout follows the Tattu specification and prior project decoder. The
incrementing sequence and payload-only CRC follow William's photographed
implementation. This module implements those operations alongside freshness;
do not additionally decode the same frames in a second decoder.

For separate physical buses, the intended application wiring is:

```cpp
Can motor_can;
Can battery_can;
BmsInterface bms(battery_can);

// After verifying controller availability, board pins, wiring and bitrates:
// ESP_ERROR_CHECK(motor_can.begin(motor_tx, motor_rx, motor_bitrate));
// ESP_ERROR_CHECK(battery_can.begin(battery_tx, battery_rx, battery_bitrate));

Can::Frame frame;
const esp_err_t result = bms.receive(frame); // nonblocking
if (result == ESP_OK) {
    const auto decoded = bms.processFrame(frame, esp_timer_get_time());
    if (decoded == BmsInterface::Result::CrcMismatch) {
        // Log the failure; the previous sample was not refreshed.
    }
}

// Do this every control iteration, including when no new frames arrive.
const auto state = bms.state(esp_timer_get_time());
if (state == BmsInterface::State::Fresh) {
    const auto *status = bms.latest();
    const bool low_cell = status->hasFault(BmsInterface::Fault::CellUndervoltage);
    // Feed status and low_cell into the vehicle state machine.
} else {
    // Report Missing/Stale to the vehicle state machine.
}
```

Each bus needs its own available CAN controller and external transceiver.
No pins, bitrate, second controller or new task are activated by this module.
Initialize and stop the buses from application code. Keep one receive consumer
per bus; do not attach this receiver to a motor bus already consumed elsewhere.
For a shared bus, use one dispatcher instead of competing receive consumers.

The existing source glob includes `bms_interface.cpp`. Normal driving now has
an optional separate-bus battery task, disabled by default with both GPIOs
unset. See [DRIVING.md](DRIVING.md) before enabling it. ESP32 build and hardware
verification are still required.

Run software tests without ESP-IDF or battery hardware:

```sh
python3 test/run_bms_host.py
```

These compile the actual BMS source using a stub CAN transport. They check
synthetic packet decoding, CRC corruption, sequence and toggle errors, timeouts,
invalid lengths, remote frames, freshness boundaries, fault bits and recovery.
