#pragma once

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "driver/twai.h"

#define CUBEMARS_TELEMETRY_DLC 8
#define CUBEMARS_CMD_SET_RPM 0x02
#define CUBEMARS_CMD_STATUS 0x09

typedef struct
{
    float position_deg;
    int32_t motor_erpm;
    float current_a;
    int8_t temperature_c;
    uint8_t error_code;
} cubemars_telemetry_t;

twai_message_t cubemars_encode_rpm(uint8_t motor_id, int32_t rpm);

uint32_t cubemars_telemetry_can_id(uint8_t motor_id);

bool cubemars_decode_telemetry(const uint8_t *data, size_t dlc, int gear_ratio, cubemars_telemetry_t *out);

const char *cubemars_describe_error(uint8_t error_code);
