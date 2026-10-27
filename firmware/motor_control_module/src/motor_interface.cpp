#include "motor_interface.hpp"

#include <cstring>

#include "config.h"

static Can::Frame make_frame(uint32_t command, uint8_t motor_id, int32_t value)
{
  Can::Frame frame{};
  frame.id = (command << 8) | motor_id;
  frame.extended = true;
  frame.dlc = 4;
  std::memcpy(frame.data, &value, sizeof(value));
  return frame;
}

Can::Frame encode_rpm(uint8_t motor_id, int32_t rpm)
{
  return make_frame(config::MOTOR_COMMAND, motor_id, rpm);
}
