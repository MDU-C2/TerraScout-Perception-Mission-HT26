# Driving and fault handling

## Receiver-only test

Keep motor power disconnected. Build/upload this separate application, then
open the interactive serial monitor (no shell pipes):

```sh
~/.platformio/penv/bin/pio run -e rc-monitor -t upload
~/.platformio/penv/bin/pio device monitor -b 115200
```

It prints throttle, steering, requested mode, neutral status and radio faults
twice per second, including when frames stop. It never initializes CAN or sends
motor commands. Uploading replaces the driving application; restore it with
`pio run -e esp32-c6-devkitm-1 -t upload` before driving. Exit with Control+C.
Neutral uses the current configured midpoint/deadband; this test does not
automatically calibrate or change them.
The current neutral band is 992 ±60 counts (932 through 1052 inclusive)
on both throttle and steering. This band also produces zero manual input.

The RadioMaster uses CH3 (index 2) for throttle and CH1 (index 0) for steering.
Motor CAN runs at 1 Mbit/s on configured GPIO5 TX / GPIO4 RX. Motor IDs are
1/3 left, 2/4 right. Speed requests are limited to 5,000 ERPM (about 0.485 m/s with the configured
21 pole pairs, 9:1 gearing and 0.175 m wheel radius).

## Enabling motion

1. All four motors must report extended status frames 0x2901..0x2904, DLC 8,
   without a nonzero fault byte (byte 7). Missing/stale feedback inhibits driving;
   the configured feedback timeout is 500 ms. Verify motor status publishing.
2. Select disabled and centre throttle and steering. Then select manual.
3. After signal loss, a motor fault, bus-off, or command failure, repeat the
   disable/neutral step. Motion does not re-enable simply because signal returns.
4. To resume autonomous driving after manual takeover: disable, centre sticks,
   then select autonomous. Fresh NUC commands are required (200 ms timeout).

Motor commands are nonblocking. A driver-enqueued frame is not proof that a
motor received or executed it. CAN slots are retained until their exact TX-done
callback; if a driver fails to complete a frame, that slot stays unavailable
rather than being overwritten. Verify bus-off behavior on the actual ESP-IDF.
Controller command watchdogs and independent emergency stopping remain required;
zero ERPM is not torque disable. The control loop runs every 50 ms.

The NUC link is 115200 baud, 8 data bits, even parity, one stop bit (8E1), UART0
GPIO17 TX / GPIO18 RX. Match these settings at the NUC. Logging is configured
for the native USB Serial/JTAG port, so it does not share UART0 with the NUC.
Use the ESP32 native USB port for the serial monitor.

## Separate battery bus

Battery support is prepared in main.cpp but `BATTERY_CAN_ENABLED` defaults to
false. No battery protection is active in that configuration. Both battery GPIOs
are deliberately unset (-1). Confirm suitable board pins, wiring, transceiver,
termination and the second CAN-controller availability before enabling it.
Do not choose pins just from the software without checking the Waveshare pinout.

When enabled, a separate task feeds BmsInterface and publishes status to the
control task. Missing/stale data or any reported fault inhibits driving; recovery
requires disable/neutral. No extra voltage/current thresholds are guessed.
CRC coverage and real battery compatibility still require capture validation.

## Host checks

From this firmware folder:

```sh
python3 test/run_can_host.py
python3 test/run_drive_host.py
python3 test/run_bms_host.py
```

These mock ESP-IDF transport/clock calls. They do not replace an ESP32 build or
physical validation of signs, pin assignments, feedback, watchdogs, or stopping.

## Driving display

The normal driving application prints one DRIVE summary twice per second:
BLOCKED/ENABLED, the current reason, requested SB mode, raw throttle (T),
steering (S), neutral status and feedback for motor IDs 1/3/2/4 in that order.
Y means fresh feedback without a fault, F means fresh feedback with a fault,
and - means missing/stale. ENABLED means commands are permitted, not proof
that wheels are moving or stopped. Detailed SBUS channel values, frame age and receiver counters are also printed
once per second. CAN send failures include the motor ID and driver error.
Each actual mode transition is logged immediately; disarming includes RC
freshness/flags, battery/bus status and each motor feedback age/fault byte.
Capture the MODE transition and its following motor lines when investigating
an unexpected stop; the periodic summary alone may miss brief faults.

Manual driving now ramps both wheel commands together: approximately one second
from zero to full configured speed, two seconds for full forward to full reverse.
Pivot turns remain available. Returning both mixed commands to zero, disabling,
or faults bypass/reset the ramp. Actual physical stopping depends on the motor
controller; autonomous commands retain their existing upstream motion profile.

## Persistent CAN transmit diagnostics

On a persistent send failure, TX DIAG reports occupied persistent slots,
accepted/completed/failed callbacks, unmatched frame callbacks, and timeout
counts split into slot exhaustion, mutex acquisition and driver enqueue.
BUS DIAG reports driver state, TX/RX error counters, free driver queue entries,
cumulative bus errors and dropped RX frames. Save two consecutive snapshots.
Healthy motor feedback proves RX traffic; it does not prove successful TX.
These diagnostics do not change speed, retry policy or automatically restart
CAN. Keep SB OFF while investigating persistent transmission failures.
