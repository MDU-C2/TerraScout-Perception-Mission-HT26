#include "motor_interface.hpp"

#include "config.h"

static Can::Frame make_frame(uint32_t command, uint8_t motor_id, int32_t value)
{
  Can::Frame frame{};
  frame.id = (command << 8) | motor_id;
  frame.extended = true;
  frame.dlc = 4;
  // CubeMars commands use a signed 32-bit value, most significant byte first.
  const uint32_t raw = static_cast<uint32_t>(value);
  frame.data[0] = static_cast<uint8_t>(raw >> 24);
  frame.data[1] = static_cast<uint8_t>(raw >> 16);
  frame.data[2] = static_cast<uint8_t>(raw >> 8);
  frame.data[3] = static_cast<uint8_t>(raw);
  return frame;
}

Can::Frame encode_rpm(uint8_t motor_id, int32_t rpm)
{
  return make_frame(config::MOTOR_COMMAND, motor_id, rpm);
}
