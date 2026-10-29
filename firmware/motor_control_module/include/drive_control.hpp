#pragma once

#include <cstdint>

#include "can.hpp"
#include "sbus.hpp"

enum class DriveMode : uint8_t
{
  DISABLED,
  MANUAL,
  AUTONOMOUS,
};

struct ControllerInput
{
  SbusFrame frame;
  int64_t received_at_us = 0;
};

struct NucInput
{
  // Signed motor-rotor electrical RPM, already in physical motor direction.
  // With the configured installation, forward uses negative left / positive
  // right ERPM. The NUC must apply this sign convention exactly once.
  int32_t left_erpm = 0;
  int32_t right_erpm = 0;
  int64_t received_at_us = 0;
};

struct SendFailure
{
  uint8_t motor_id = 0;
  esp_err_t error = ESP_OK;
};

struct WheelVelocities
{
  double left_rad_s = 0.0;
  double right_rad_s = 0.0;
};

class ManualRamp {
public:
  WheelVelocities update(WheelVelocities target, double elapsed_s);
  void reset() { current_ = {}; }
private:
  WheelVelocities current_{};
};

// Pure throttle/steering conversion. Invalid channel values return false and
// zero both outputs. Caller still checks receiver freshness/failsafe separately.
bool manual_wheel_velocities(uint16_t throttle, uint16_t steering,
                             WheelVelocities & output);

DriveMode mode_from_controller(const ControllerInput & input);
bool nuc_input_fresh(const NucInput & input, int64_t now_us);

// Startup/reconnection requires disable + neutral. Manual -> autonomous also
// requires visiting disable; toggling to autonomous is the explicit resume.
class DriveEnableGate {
public:
  DriveMode update(const ControllerInput & input);
  void reset() { enabled_ = false; previous_ = DriveMode::DISABLED; }
private:
  bool enabled_ = false;
  DriveMode previous_ = DriveMode::DISABLED;
};
const char * mode_name(DriveMode mode);
double sbus_to_wheel_velocity(uint16_t value);
bool send_drive(Can & can, double left_velocity_rad_s, double right_velocity_rad_s,
                SendFailure & first_failure);
// Physical motor ERPM: clamps both sides, but does not invert either side.
// send_drive() instead takes wheel rad/s and applies the installation signs.
bool send_drive_erpm(Can & can, int32_t left_erpm, int32_t right_erpm,
                     SendFailure & first_failure);
