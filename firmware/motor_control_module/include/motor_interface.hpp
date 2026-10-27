#pragma once

#include "can.hpp"

Can::Frame encode_rpm(uint8_t motor_id, int32_t rpm);
