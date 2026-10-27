#include "drive_control.hpp"

#include <algorithm>
#include <cmath>

#include "esp_log.h"
#include "esp_timer.h"
#include "motor_interface.hpp"

namespace {

constexpr char TAG[] = "DRIVE";
constexpr double MAX_WHEEL_VELOCITY_RAD_S =
  config::MAX_MOTOR_ERPM * (config::TWO_PI / 60.0) /
  (config::POLE_PAIRS * config::GEAR_RATIO);

int32_t velocity_to_erpm(double velocity_rad_s, bool negate)
{
  if (!std::isfinite(velocity_rad_s)) {
    velocity_rad_s = 0.0;
  }
  if (negate) {
    velocity_rad_s = -velocity_rad_s;
  }

  return static_cast<int32_t>(std::clamp(
    velocity_rad_s * (60.0 / config::TWO_PI) * config::POLE_PAIRS * config::GEAR_RATIO,
    -static_cast<double>(config::MAX_MOTOR_ERPM),
    static_cast<double>(config::MAX_MOTOR_ERPM)));
}

}  // namespace

double sbus_to_wheel_velocity(uint16_t value)
{
  const int32_t centered = static_cast<int32_t>(value) - config::SBUS_CHANNEL_MID;
  const int32_t span = centered >= 0 ?
    config::SBUS_CHANNEL_MAX - config::SBUS_CHANNEL_MID :
    config::SBUS_CHANNEL_MID - config::SBUS_CHANNEL_MIN;
  return std::clamp(
    static_cast<double>(centered) / static_cast<double>(span) * MAX_WHEEL_VELOCITY_RAD_S,
    -MAX_WHEEL_VELOCITY_RAD_S, MAX_WHEEL_VELOCITY_RAD_S);
}

const char * mode_name(DriveMode mode)
{
  switch (mode) {
    case DriveMode::DISABLED:
      return "DISABLED";
    case DriveMode::MANUAL:
      return "MANUAL";
    case DriveMode::AUTONOMOUS:
      return "AUTONOMOUS";
  }
  return "UNKNOWN";
}

DriveMode mode_from_controller(const ControllerInput & input)
{
  if (
    !input.frame.valid || input.frame.frame_lost || input.frame.failsafe ||
    esp_timer_get_time() - input.received_at_us > config::CONTROLLER_INPUT_TIMEOUT_US) {
    return DriveMode::DISABLED;
  }

  const uint16_t sw = input.frame.channels[config::SBUS_MODE_CHANNEL];
  if (sw < 600) {
    return DriveMode::DISABLED;
  }
  if (sw < 1400) {
    return DriveMode::MANUAL;
  }
  if (sw < 2000) {
    return DriveMode::AUTONOMOUS;
  }
  return DriveMode::DISABLED;
}

bool send_drive_erpm(
  Can & can, int32_t left_erpm, int32_t right_erpm, SendFailure & first_failure)
{
  const uint8_t ids[] = {
    config::MOTOR_FRONT_LEFT, config::MOTOR_REAR_LEFT,
    config::MOTOR_FRONT_RIGHT, config::MOTOR_REAR_RIGHT};
  const int32_t erpms[] = {left_erpm, left_erpm, right_erpm, right_erpm};
  bool send_failed = false;

  for (size_t i = 0; i < 4; ++i) {
    const esp_err_t error = can.send(encode_rpm(ids[i], erpms[i]), 100);
    if (error == ESP_OK) {
      continue;
    }

    if (!send_failed) {
      first_failure.motor_id = ids[i];
      first_failure.error = error;
    }
    send_failed = true;
    ESP_LOGW(
      TAG, "Failed to send motor ID %u command eRPM=%ld; cause: %s",
      ids[i], static_cast<long>(erpms[i]), esp_err_to_name(error));
  }
  return !send_failed;
}

bool send_drive(
  Can & can, double left_velocity_rad_s, double right_velocity_rad_s,
  SendFailure & first_failure)
{
  return send_drive_erpm(
    can, velocity_to_erpm(left_velocity_rad_s, true),
    velocity_to_erpm(right_velocity_rad_s, false), first_failure);
}
