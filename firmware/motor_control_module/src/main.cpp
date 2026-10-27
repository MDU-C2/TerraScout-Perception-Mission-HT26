#include <cstdint>
#include <cstring>

#include "can.hpp"
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

void sbus_task(void *)
{
  ESP_LOGI(
    "SBUS", "Starting SBUS task on UART%d RX GPIO%d at %d baud",
    static_cast<int>(config::SBUS_UART), static_cast<int>(config::SBUS_RX_PIN),
    config::SBUS_BAUD);
  Sbus sbus(config::SBUS_UART, config::SBUS_RX_PIN);
  SbusFrame frame{};
  ControllerInput input;
  int64_t last_status_us = esp_timer_get_time();

  while (true) {
    if (sbus.read(frame, pdMS_TO_TICKS(100))) {
      input.frame = frame;
      input.received_at_us = sbus.last_frame_time_us();
      xQueueOverwrite(rc_input_queue, &input);
      ESP_LOGI(
        "SBUS", "Received frame: ch1=%u ch2=%u ch3=%u ch4=%u ch5=%u ch6=%u "
        "ch7=%u ch8=%u ch9=%u ch10=%u ch11=%u ch12=%u ch13=%u ch14=%u "
        "ch15=%u ch16=%u ch17=%d ch18=%d lost=%d failsafe=%d",
        frame.channels[0], frame.channels[1], frame.channels[2],
        frame.channels[3], frame.channels[4], frame.channels[5],
        frame.channels[6], frame.channels[7], frame.channels[8],
        frame.channels[9], frame.channels[10], frame.channels[11],
        frame.channels[12], frame.channels[13], frame.channels[14],
        frame.channels[15], frame.ch17, frame.ch18, frame.frame_lost,
        frame.failsafe);
      last_status_us = esp_timer_get_time();
    } else {
      const int64_t now_us = esp_timer_get_time();
      if (now_us - last_status_us >= 1000000) {
        ESP_LOGW(
          "SBUS", "No valid frame received; good_frames=%lu bad_windows=%lu",
          static_cast<unsigned long>(sbus.good_frames()),
          static_cast<unsigned long>(sbus.bad_windows()));
        last_status_us = now_us;
      }
    }
  }
}

void nuc_task(void *)
{
  Uart uart(config::NUC_UART, config::NUC_TX_PIN, config::NUC_RX_PIN, config::NUC_BAUDRATE);
  uint8_t packet[config::NUC_PACKET_SIZE] = {};
  NucInput input;
  int64_t last_invalid_log_us = 0;

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
    ESP_LOGI(
      "NUC", "Received command left=%ld eRPM right=%ld eRPM",
      static_cast<long>(input.left_erpm), static_cast<long>(input.right_erpm));
  }
}

void control_task(void *)
{
  Can can;
  if (can.begin(config::CAN_TX_PIN, config::CAN_RX_PIN, config::CAN_BITRATE) != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize CAN");
    return;
  }

  ControllerInput controller{};
  NucInput nuc{};
  DriveMode mode = DriveMode::DISABLED;
  int64_t send_failure_started_us = 0;
  int64_t last_recovery_attempt_us = 0;
  TickType_t last_wake = xTaskGetTickCount();
  const TickType_t control_period = pdMS_TO_TICKS(50);

  while (true) {
    if (xQueueReceive(rc_input_queue, &controller, 0) == pdTRUE) {
      mode = mode_from_controller(controller);
    }

    ESP_LOGI(TAG, "Current drive mode: %s", mode_name(mode));

    bool send_succeeded = true;
    SendFailure first_failure;
    switch (mode) {
      case DriveMode::DISABLED:
        send_succeeded = send_drive(can, 0.0, 0.0, first_failure);
        break;
      case DriveMode::MANUAL:
        send_succeeded = send_drive(
          can,
          sbus_to_wheel_velocity(controller.frame.channels[2]),
          sbus_to_wheel_velocity(controller.frame.channels[0]),
          first_failure);
        break;
      case DriveMode::AUTONOMOUS:
        if (xQueueReceive(nuc_input_queue, &nuc, 0) == pdTRUE) {
          send_succeeded = send_drive_erpm(
            can, nuc.left_erpm, nuc.right_erpm, first_failure);
        } else {
          ESP_LOGW(TAG, "No autonomous command received");
          mode = DriveMode::DISABLED;
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
      send_failure_started_us = now_us;
    } else if (
      now_us - send_failure_started_us >= config::CAN_SEND_FAILURE_TIMEOUT_US) {
        ESP_LOGE(
          "CAN",
          "Motor command transmission failed for motor ID 0x%02X; cause: %s; "
          "failure persisted for %lld us",
          first_failure.motor_id, esp_err_to_name(first_failure.error),
          static_cast<long long>(now_us - send_failure_started_us));
      mode = DriveMode::DISABLED;
      send_failure_started_us = 0;
    }

    vTaskDelayUntil(&last_wake, control_period);
  }
}

}  // namespace

extern "C" void app_main(void)
{
  rc_input_queue = xQueueCreate(1, sizeof(ControllerInput));
  nuc_input_queue = xQueueCreate(1, sizeof(NucInput));
  if (rc_input_queue == nullptr || nuc_input_queue == nullptr) {
    ESP_LOGE(TAG, "Failed to create input queues");
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
