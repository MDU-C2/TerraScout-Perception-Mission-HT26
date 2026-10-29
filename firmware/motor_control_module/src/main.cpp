#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "can.hpp"
#include "bms_interface.hpp"
#include "drive_control.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "nuc_protocol.hpp"
#include "sbus.hpp"
#include "uart.hpp"

namespace {

constexpr char TAG[] = "MAIN";
QueueHandle_t rc_input_queue = nullptr;
QueueHandle_t nuc_input_queue = nullptr;
struct BatteryInput { bool fresh = false; uint32_t faults = 0; int64_t received_us = 0; };
QueueHandle_t battery_input_queue = nullptr;

void battery_task(void *)
{
  if (config::BATTERY_CAN_TX_PIN < 0 || config::BATTERY_CAN_RX_PIN < 0) {
    ESP_LOGE("BMS", "Battery CAN pins are not configured");
    vTaskDelete(nullptr);
    return;
  }
  Can can;
  if (can.begin(config::BATTERY_CAN_TX_PIN, config::BATTERY_CAN_RX_PIN,
                config::BATTERY_CAN_BITRATE, 64) != ESP_OK) {
    ESP_LOGE("BMS", "Battery CAN initialization failed");
    vTaskDelete(nullptr);
    return;
  }
  BmsInterface bms(can);
  while (true) {
    Can::Frame frame;
    if (can.receive(frame, 20) == ESP_OK) bms.processFrame(frame, esp_timer_get_time());
    BatteryInput input;
    input.fresh = bms.state(esp_timer_get_time()) == BmsInterface::State::Fresh;
    if (const auto *status = bms.latest()) {
      input.faults = status->fault_flags;
      input.received_us = status->received_us;
    }
    xQueueOverwrite(battery_input_queue, &input);
  }
}

void sbus_task(void *)
{
  ESP_LOGI(
    "SBUS", "Starting SBUS task on UART%d RX GPIO%d at %d baud",
    static_cast<int>(config::SBUS_UART), static_cast<int>(config::SBUS_RX_PIN),
    config::SBUS_BAUD);
  Sbus sbus(config::SBUS_UART, config::SBUS_RX_PIN);
  SbusFrame frame{};
  ControllerInput input;
  int64_t last_radio_log_us = 0;
  while (true) {
    if (sbus.read(frame, pdMS_TO_TICKS(100))) {
      input.frame = frame;
      input.received_at_us = sbus.last_frame_time_us();
      xQueueOverwrite(rc_input_queue, &input);
    }
    const int64_t now = esp_timer_get_time();
    if (now - last_radio_log_us >= 1000000) {
      last_radio_log_us = now;
      ESP_LOGI("SBUS", "ch1=%u ch2=%u ch3=%u ch4=%u ch5=%u ch6=%u ch7=%u ch8=%u "
        "ch9=%u ch10=%u ch11=%u ch12=%u ch13=%u ch14=%u ch15=%u ch16=%u "
        "valid=%d lost=%d failsafe=%d age_us=%lld good=%lu bad=%lu",
        frame.channels[0], frame.channels[1], frame.channels[2], frame.channels[3],
        frame.channels[4], frame.channels[5], frame.channels[6], frame.channels[7],
        frame.channels[8], frame.channels[9], frame.channels[10], frame.channels[11],
        frame.channels[12], frame.channels[13], frame.channels[14], frame.channels[15],
        frame.valid, frame.frame_lost, frame.failsafe,
        static_cast<long long>(sbus.last_frame_time_us() > 0 ? now-sbus.last_frame_time_us() : -1),
        static_cast<unsigned long>(sbus.good_frames()), static_cast<unsigned long>(sbus.bad_windows()));
    }
  }
}

void nuc_task(void *)
{
  Uart uart(config::NUC_UART, config::NUC_TX_PIN, config::NUC_RX_PIN, config::NUC_BAUDRATE);
  uint8_t packet[config::NUC_PACKET_SIZE] = {};
  NucInput input;
  int64_t last_invalid_log_us = 0;
  int64_t last_command_log_us = 0;

  while (true) {
    if (uart.receive_exact(packet, 1, pdMS_TO_TICKS(100)) != 1 ||
      packet[0] != config::NUC_START_BYTE) {
      continue;
    }
    if (
      uart.receive_exact(packet + 1, config::NUC_PACKET_SIZE - 1, pdMS_TO_TICKS(20)) !=
        config::NUC_PACKET_SIZE - 1 ||
      nuc_checksum(packet) != packet[config::NUC_PACKET_SIZE - 1]) {
      const int64_t now = esp_timer_get_time();
      if (now - last_invalid_log_us >= 1000000) {
        ESP_LOGW("NUC", "Invalid autonomous command packet");
        last_invalid_log_us = now;
      }
      continue;
    }

    std::memcpy(&input.left_erpm, packet + 1, sizeof(input.left_erpm));
    std::memcpy(&input.right_erpm, packet + 5, sizeof(input.right_erpm));
    input.received_at_us = esp_timer_get_time();
    xQueueOverwrite(nuc_input_queue, &input);
    if (input.received_at_us - last_command_log_us >= 1000000) {
    ESP_LOGI(
      "NUC", "Received command left=%ld eRPM right=%ld eRPM",
      static_cast<long>(input.left_erpm), static_cast<long>(input.right_erpm));
    last_command_log_us = input.received_at_us;
    }
  }
}

void control_task(void *)
{
  Can can;
  if (can.begin(config::CAN_TX_PIN, config::CAN_RX_PIN, config::CAN_BITRATE) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize CAN");
    vTaskDelete(nullptr);
    return;
  }

  ControllerInput controller{};
  NucInput nuc{};
  DriveMode mode = DriveMode::DISABLED;
  DriveEnableGate enable_gate;
  ManualRamp manual_ramp;
  int64_t previous_control_us = esp_timer_get_time();
  int64_t motor_received[4]{};
  uint8_t motor_errors[4]{};
  const uint8_t motor_ids[] = {config::MOTOR_FRONT_LEFT, config::MOTOR_REAR_LEFT,
                               config::MOTOR_FRONT_RIGHT, config::MOTOR_REAR_RIGHT};
  int64_t last_display_us = 0;
  BatteryInput battery{};
  int64_t send_failure_started_us = 0;
  int64_t last_recovery_attempt_us = 0;
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t control_period = pdMS_TO_TICKS(50);

  while (true) {
    const DriveMode previous_mode = mode;
    const char *stop_reason = nullptr;
    xQueueReceive(rc_input_queue, &controller, 0);
    const int64_t check_us = esp_timer_get_time();
    const double elapsed_s = (check_us - previous_control_us) / 1000000.0;
    previous_control_us = check_us;
    // Bounded draining keeps bus traffic from monopolizing the control task.
    Can::Frame feedback;
    for (unsigned n = 0; n < 32 && can.receive(feedback, 0) == ESP_OK; ++n) {
      if (!feedback.extended || feedback.rtr || feedback.dlc != 8) continue;
      for (unsigned i = 0; i < 4; ++i) {
        if (feedback.id == ((0x29U << 8) | motor_ids[i])) {
          motor_received[i] = check_us;
          motor_errors[i] = feedback.data[7];
        }
      }
    }
    bool motor_fault = can.isBusOff();
    for (unsigned i = 0; i < 4; ++i) {
      motor_fault = motor_fault || motor_errors[i] != 0 || motor_received[i] == 0 ||
        check_us - motor_received[i] >= config::MOTOR_FEEDBACK_TIMEOUT_US;
    }
    xQueueReceive(battery_input_queue, &battery, 0);
    const bool battery_fault = config::BATTERY_CAN_ENABLED &&
      (!battery.fresh || battery.faults != 0 || battery.received_us <= 0 ||
       check_us < battery.received_us || check_us - battery.received_us >= 1000000);
    if (motor_fault || battery_fault) enable_gate.reset();
    mode = (motor_fault || battery_fault) ? DriveMode::DISABLED : enable_gate.update(controller);
    if (previous_mode != DriveMode::DISABLED && mode == DriveMode::DISABLED) {
      const int64_t age = check_us - controller.received_at_us;
      stop_reason = can.isBusOff() ? "CAN BUS OFF" : motor_fault ? "MOTOR FEEDBACK/FAULT" :
        battery_fault ? "BATTERY" : !controller.frame.valid ? "INVALID RC FRAME" :
        controller.frame.failsafe ? "RC FAILSAFE" : controller.frame.frame_lost ? "RC FRAME LOST" :
        controller.received_at_us <= 0 || age < 0 || age >= config::CONTROLLER_INPUT_TIMEOUT_US ?
        "RC TIMEOUT" : "MODE SWITCH / REARM REQUIRED";
    }
    if (mode != DriveMode::MANUAL) manual_ramp.reset();
    // Cache the newest NUC packet; receiving nothing does not erase a command.
    xQueueReceive(nuc_input_queue, &nuc, 0);

    bool send_succeeded = true;
    SendFailure first_failure;
    switch (mode) {
      case DriveMode::DISABLED:
        send_succeeded = send_drive(can, 0.0, 0.0, first_failure);
        break;
      case DriveMode::MANUAL: {
        WheelVelocities wheels;
        if (!manual_wheel_velocities(
              controller.frame.channels[config::SBUS_THROTTLE_CHANNEL],
              controller.frame.channels[config::SBUS_STEERING_CHANNEL], wheels)) {
          mode = DriveMode::DISABLED;
          enable_gate.reset();
          stop_reason = "INVALID MANUAL CHANNELS";
          ESP_LOGW(TAG, "Invalid manual channel values; sending zero");
          manual_ramp.reset();
        }
        wheels = manual_ramp.update(wheels, elapsed_s);
        send_succeeded = send_drive(can, wheels.left_rad_s, wheels.right_rad_s, first_failure);
        break;
      }
      case DriveMode::AUTONOMOUS:
        if (nuc_input_fresh(nuc, esp_timer_get_time())) {
          send_succeeded = send_drive_erpm(can, nuc.left_erpm, nuc.right_erpm, first_failure);
        } else {
          stop_reason = "NUC COMMAND TIMEOUT";
          ESP_LOGW(TAG, "Autonomous command missing or stale; sending zero");
          mode = DriveMode::DISABLED;
          enable_gate.reset();
          send_succeeded = send_drive_erpm(can, 0, 0, first_failure);
        }
        break;
    }

    const int64_t now_us = esp_timer_get_time();
    if (can.isBusOff() && now_us - last_recovery_attempt_us >= 1000000) {
      ESP_LOGE(
        "CAN",
        "CAN controller is bus-off; check bitrate=%lu, TX GPIO=%d, RX GPIO=%d, "
        "transceiver power/enable, wiring, and 120 ohm termination",
        static_cast<unsigned long>(config::CAN_BITRATE),
        static_cast<int>(config::CAN_TX_PIN), static_cast<int>(config::CAN_RX_PIN));
      const esp_err_t recovery_error = can.recover();
      if (recovery_error != ESP_OK) {
        ESP_LOGE("CAN", "CAN bus recovery failed: %s", esp_err_to_name(recovery_error));
      } else {
        ESP_LOGW("CAN", "CAN bus recovery requested");
      }
      last_recovery_attempt_us = now_us;
    }

    if (send_succeeded) {
      send_failure_started_us = 0;
    } else if (send_failure_started_us == 0) {
      ESP_LOGE("CAN", "Send failed: motor=%u error=%s", first_failure.motor_id,
               esp_err_to_name(first_failure.error));
      send_failure_started_us = now_us;
    } else if (
      now_us - send_failure_started_us >= config::CAN_SEND_FAILURE_TIMEOUT_US) {
      stop_reason = "PERSISTENT CAN SEND FAILURE";
      ESP_LOGE("CAN", "Persistent send failure: motor=%u error=%s", first_failure.motor_id,
               esp_err_to_name(first_failure.error));
      can.logDiagnostics();
      mode = DriveMode::DISABLED;
      enable_gate.reset();
      send_failure_started_us = 0;
    }

    if (!send_succeeded) manual_ramp.reset();
    if (previous_mode != mode) {
      ESP_LOGW("DRIVE", "MODE %s -> %s; reason=%s; RC age_us=%lld valid=%d lost=%d failsafe=%d battery_fault=%d bus_off=%d",
        mode_name(previous_mode), mode_name(mode), stop_reason ? stop_reason : "SWITCH / ENABLE GATE",
        static_cast<long long>(now_us-controller.received_at_us), controller.frame.valid,
        controller.frame.frame_lost, controller.frame.failsafe, battery_fault, can.isBusOff());
      if (mode == DriveMode::DISABLED) {
        for (unsigned i=0; i<4; ++i) {
          ESP_LOGW("DRIVE", "Motor %u feedback_age_us=%lld fault_byte=0x%02X",
            motor_ids[i], static_cast<long long>(motor_received[i] > 0 ? now_us-motor_received[i] : -1),
            motor_errors[i]);
        }
      }
    }


    if (now_us - last_display_us >= 500000) {
      last_display_us = now_us;
      const auto throttle = controller.frame.channels[config::SBUS_THROTTLE_CHANNEL];
      const auto steering = controller.frame.channels[config::SBUS_STEERING_CHANNEL];
      const auto sw = controller.frame.channels[config::SBUS_MODE_CHANNEL];
      const bool radio_ok = controller.frame.valid && !controller.frame.failsafe &&
        !controller.frame.frame_lost && controller.received_at_us > 0 &&
        now_us >= controller.received_at_us &&
        now_us - controller.received_at_us < config::CONTROLLER_INPUT_TIMEOUT_US;
      const bool neutral =
        std::abs(static_cast<int>(throttle) - config::SBUS_CHANNEL_MID) <= config::SBUS_DEADBAND &&
        std::abs(static_cast<int>(steering) - config::SBUS_CHANNEL_MID) <= config::SBUS_DEADBAND;
      const char *reason = !radio_ok ? "RADIO LOST/FAILSAFE" : can.isBusOff() ? "CAN BUS OFF" :
        !send_succeeded ? "CAN SEND FAILED" : motor_fault ? "MOTOR FEEDBACK/FAULT" :
        battery_fault ? "BATTERY" : mode != DriveMode::DISABLED ? "ENABLED" :
        sw < 600 ? "SB DISABLED" : !neutral ? "CENTRE STICKS, THEN SB DISABLED" :
        "SELECT SB DISABLED BEFORE REARMING";
      ESP_LOGI("DRIVE", "%s | %s | SB=%s T=%u S=%u Neutral=%s | Motors 1/3/2/4=%c%c%c%c",
        mode == DriveMode::DISABLED || !send_succeeded ? "BLOCKED" : "ENABLED",
        reason, sw < 600 ? "OFF" : sw < 1400 ? "MANUAL" : sw < 2000 ? "AUTO" : "INVALID",
        static_cast<unsigned>(throttle), static_cast<unsigned>(steering), neutral ? "YES" : "NO",
        motor_received[0] > 0 && now_us - motor_received[0] < config::MOTOR_FEEDBACK_TIMEOUT_US ? (motor_errors[0] ? 'F' : 'Y') : '-',
        motor_received[1] > 0 && now_us - motor_received[1] < config::MOTOR_FEEDBACK_TIMEOUT_US ? (motor_errors[1] ? 'F' : 'Y') : '-',
        motor_received[2] > 0 && now_us - motor_received[2] < config::MOTOR_FEEDBACK_TIMEOUT_US ? (motor_errors[2] ? 'F' : 'Y') : '-',
        motor_received[3] > 0 && now_us - motor_received[3] < config::MOTOR_FEEDBACK_TIMEOUT_US ? (motor_errors[3] ? 'F' : 'Y') : '-');
    }

    vTaskDelayUntil(&last_wake, control_period);
  }
}

}  // namespace

extern "C" void app_main(void)
{
  rc_input_queue = xQueueCreate(1, sizeof(ControllerInput));
  nuc_input_queue = xQueueCreate(1, sizeof(NucInput));
  battery_input_queue = xQueueCreate(1, sizeof(BatteryInput));
  if (rc_input_queue == nullptr || nuc_input_queue == nullptr || battery_input_queue == nullptr) {
    ESP_LOGE(TAG, "Failed to create input queues");
    return;
  }

  if (config::BATTERY_CAN_ENABLED &&
      xTaskCreate(battery_task, "battery_task", 4096, nullptr, 4, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create battery task");
    return;
  }

  if (
    xTaskCreate(sbus_task, "sbus_task", 4096, nullptr, 6, nullptr) != pdPASS ||
    xTaskCreate(nuc_task, "nuc_task", 4096, nullptr, 5, nullptr) != pdPASS ||
    xTaskCreate(control_task, "control_task", 4096, nullptr, 7, nullptr) != pdPASS) {
    ESP_LOGE(TAG, "Failed to create one or more tasks");
    return;
  }
  ESP_LOGI(TAG, "All firmware tasks started");
}
