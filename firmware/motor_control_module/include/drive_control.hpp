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
  int32_t left_erpm = 0;
  int32_t right_erpm = 0;
  int64_t received_at_us = 0;
};

struct SendFailure
{
  uint8_t motor_id = 0;
  esp_err_t error = ESP_OK;
};

DriveMode mode_from_controller(const ControllerInput & input);
const char * mode_name(DriveMode mode);
double sbus_to_wheel_velocity(uint16_t value);
bool send_drive(Can & can, double left_velocity_rad_s, double right_velocity_rad_s,
                SendFailure & first_failure);
bool send_drive_erpm(Can & can, int32_t left_erpm, int32_t right_erpm,
                     SendFailure & first_failure);
