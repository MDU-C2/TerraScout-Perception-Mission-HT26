#include "drive_control.hpp"

#include <algorithm>
#include <cmath>

#include "esp_timer.h"
#include "motor_interface.hpp"

namespace {

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

WheelVelocities ManualRamp::update(WheelVelocities target, double elapsed_s)
{
  static_assert(config::MANUAL_RAMP_SECONDS > 0.0);
  if (!std::isfinite(target.left_rad_s) || !std::isfinite(target.right_rad_s) ||
      !std::isfinite(elapsed_s) || elapsed_s <= 0.0 ||
      (target.left_rad_s == 0.0 && target.right_rad_s == 0.0)) {
    reset();
    return current_;
  }
  // Bound delayed iterations so a scheduling pause cannot cause a large jump.
  const double step = MAX_WHEEL_VELOCITY_RAD_S * std::min(elapsed_s, 0.05) /
                      config::MANUAL_RAMP_SECONDS;
  const double dl = target.left_rad_s - current_.left_rad_s;
  const double dr = target.right_rad_s - current_.right_rad_s;
  const double scale = std::max(1.0, std::max(std::abs(dl), std::abs(dr)) / step);
  current_.left_rad_s += dl / scale;
  current_.right_rad_s += dr / scale;
  return current_;
}

double sbus_to_wheel_velocity(uint16_t value)
{
  static_assert(config::SBUS_CHANNEL_MIN < config::SBUS_CHANNEL_MID &&
                config::SBUS_CHANNEL_MID < config::SBUS_CHANNEL_MAX);
  static_assert(config::SBUS_DEADBAND >= 0 &&
                config::SBUS_DEADBAND < config::SBUS_CHANNEL_MID - config::SBUS_CHANNEL_MIN &&
                config::SBUS_DEADBAND < config::SBUS_CHANNEL_MAX - config::SBUS_CHANNEL_MID);
  if (value < config::SBUS_CHANNEL_MIN || value > config::SBUS_CHANNEL_MAX) {
    return 0.0;
  }
  const int32_t centered = static_cast<int32_t>(value) - config::SBUS_CHANNEL_MID;
  if (std::abs(centered) <= config::SBUS_DEADBAND) {
    return 0.0;
  }
  const int32_t span = centered >= 0 ?
    config::SBUS_CHANNEL_MAX - config::SBUS_CHANNEL_MID :
    config::SBUS_CHANNEL_MID - config::SBUS_CHANNEL_MIN;
  // Rescale outside the deadband so there is no jump at its edge.
  const int32_t adjusted = centered > 0 ? centered - config::SBUS_DEADBAND :
                                          centered + config::SBUS_DEADBAND;
  return std::clamp(
    static_cast<double>(adjusted) / (span - config::SBUS_DEADBAND) * MAX_WHEEL_VELOCITY_RAD_S,
    -MAX_WHEEL_VELOCITY_RAD_S, MAX_WHEEL_VELOCITY_RAD_S);
}

bool manual_wheel_velocities(uint16_t throttle, uint16_t steering,
                             WheelVelocities & output)
{
  output = {};
  static_assert(config::SBUS_STEERING_SIGN == 1 || config::SBUS_STEERING_SIGN == -1);
  if (throttle < config::SBUS_CHANNEL_MIN || throttle > config::SBUS_CHANNEL_MAX ||
      steering < config::SBUS_CHANNEL_MIN || steering > config::SBUS_CHANNEL_MAX) {
    return false;
  }
  const double forward = sbus_to_wheel_velocity(throttle);
  const double turn = sbus_to_wheel_velocity(steering) * config::SBUS_STEERING_SIGN;
  double left = forward + turn;
  double right = forward - turn;
  // Scale both sides together to keep their ratio within the existing limit.
  const double scale = std::max(1.0, std::max(std::abs(left), std::abs(right)) /
                                       MAX_WHEEL_VELOCITY_RAD_S);
  output.left_rad_s = left / scale;
  output.right_rad_s = right / scale;
  return true;
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
  const int64_t now_us = esp_timer_get_time();
  if (
    !input.frame.valid || input.frame.frame_lost || input.frame.failsafe ||
    input.received_at_us <= 0 || now_us < input.received_at_us ||
    now_us - input.received_at_us >= config::CONTROLLER_INPUT_TIMEOUT_US) {
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

bool nuc_input_fresh(const NucInput & input, int64_t now_us)
{
  return input.received_at_us > 0 && now_us >= input.received_at_us &&
         now_us - input.received_at_us < config::NUC_INPUT_TIMEOUT_US;
}

DriveMode DriveEnableGate::update(const ControllerInput & input)
{
  const auto now = esp_timer_get_time();
  if (!input.frame.valid || input.frame.failsafe || input.frame.frame_lost ||
      input.received_at_us <= 0 || now < input.received_at_us ||
      now - input.received_at_us >= config::CONTROLLER_INPUT_TIMEOUT_US) {
    reset();
    return DriveMode::DISABLED;
  }
  const auto requested = mode_from_controller(input);
  const auto sw = input.frame.channels[config::SBUS_MODE_CHANNEL];
  if (requested == DriveMode::DISABLED) {
    const auto throttle = input.frame.channels[config::SBUS_THROTTLE_CHANNEL];
    const auto steering = input.frame.channels[config::SBUS_STEERING_CHANNEL];
    // Do not arm from invalid switch values or out-of-range stick values.
    enabled_ = sw >= config::SBUS_CHANNEL_MIN && sw < 600 &&
      throttle >= config::SBUS_CHANNEL_MIN && throttle <= config::SBUS_CHANNEL_MAX &&
      steering >= config::SBUS_CHANNEL_MIN && steering <= config::SBUS_CHANNEL_MAX &&
      std::abs(int(throttle) - int(config::SBUS_CHANNEL_MID)) <= config::SBUS_DEADBAND &&
      std::abs(int(steering) - int(config::SBUS_CHANNEL_MID)) <= config::SBUS_DEADBAND;
  } else if (previous_ == DriveMode::MANUAL && requested == DriveMode::AUTONOMOUS) {
    enabled_ = false;
  }
  previous_ = requested;
  return enabled_ ? requested : DriveMode::DISABLED;
}

bool send_drive_erpm(
  Can & can, int32_t left_erpm, int32_t right_erpm, SendFailure & first_failure)
{
  // Final common output boundary: applies to both NUC and manual commands.
  static_assert(config::MAX_MOTOR_ERPM > 0, "Motor ERPM limit must be positive");
  left_erpm = std::clamp(left_erpm, -config::MAX_MOTOR_ERPM, config::MAX_MOTOR_ERPM);
  right_erpm = std::clamp(right_erpm, -config::MAX_MOTOR_ERPM, config::MAX_MOTOR_ERPM);
  const uint8_t ids[] = {
    config::MOTOR_FRONT_LEFT, config::MOTOR_REAR_LEFT,
    config::MOTOR_FRONT_RIGHT, config::MOTOR_REAR_RIGHT};
  const int32_t erpms[] = {left_erpm, left_erpm, right_erpm, right_erpm};
  bool send_failed = false;

  for (size_t i = 0; i < 4; ++i) {
    const esp_err_t error = can.send(encode_rpm(ids[i], erpms[i]), 0);
    if (error == ESP_OK) {
      continue;
    }

    if (!send_failed) {
      first_failure.motor_id = ids[i];
      first_failure.error = error;
    }
    send_failed = true;
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
