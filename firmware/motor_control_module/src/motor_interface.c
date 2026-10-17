#include "motor_interface.h"
#include <string.h>

#define DEG_PER_POSITION_UNIT 0.1f
#define ERPM_PER_SPEED_UNIT 10
#define AMPERE_PER_CURRENT_UNIT 0.01f

static uint16_t read_be_u16(const uint8_t *data)
{
    return (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
}

static uint32_t read_be_u32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) |
           ((uint32_t)data[1] << 16) |
           ((uint32_t)data[2] << 8) |
           ((uint32_t)data[3]);
}

static void write_be_u32(uint8_t *data, uint32_t value)
{
    data[0] = (uint8_t)((value >> 24) & 0xFF);
    data[1] = (uint8_t)((value >> 16) & 0xFF);
    data[2] = (uint8_t)((value >> 8) & 0xFF);
    data[3] = (uint8_t)(value & 0xFF);
}

twai_message_t cubemars_encode_rpm(uint8_t motor_id, int32_t rpm)
{
    twai_message_t msg;
    memset(&msg, 0, sizeof(msg));

    msg.identifier = ((uint32_t)CUBEMARS_CMD_SET_RPM << 8) | motor_id;
    msg.extd = 1;
    msg.rtr = 0;
    msg.ss = 0;
    msg.self = 0;
    msg.dlc_non_comp = 0;
    msg.data_length_code = 4;

    write_be_u32(msg.data, (uint32_t)rpm);
    return msg;
}

uint32_t cubemars_telemetry_can_id(uint8_t motor_id)
{
    return ((uint32_t)CUBEMARS_CMD_STATUS << 8) | motor_id;
}

bool cubemars_decode_telemetry(const uint8_t *data, size_t dlc, int gear_ratio, cubemars_telemetry_t *out)
{
    (void)gear_ratio;

    if (!data || !out)
        return false;
    if (dlc != CUBEMARS_TELEMETRY_DLC)
        return false;

    out->position_deg = (float)((int16_t)read_be_u16(&data[0])) * DEG_PER_POSITION_UNIT;
    out->motor_erpm = (int32_t)((int16_t)read_be_u16(&data[2])) * ERPM_PER_SPEED_UNIT;
    out->current_a = (float)((int16_t)read_be_u16(&data[4])) * AMPERE_PER_CURRENT_UNIT;
    out->temperature_c = (int8_t)data[6];
    out->error_code = data[7];
    return true;
}

const char *cubemars_describe_error(uint8_t error_code)
{
    switch (error_code)
    {
    case 0:
        return "MOTOR OK";
    case 1:
        return "MOTOR OVER TEMPERATURE";
    case 2:
        return "MOTOR OVER CURRENT";
    case 3:
        return "MOTOR OVER VOLTAGE";
    case 4:
        return "MOTOR UNDER VOLTAGE";
    case 5:
        return "ENCODER FAULT";
    case 6:
        return "MOSFET OVER TEMPERATURE";
    case 7:
        return "MOTOR STALL";
    default:
        return "UNKNOWN ERROR";
    }
}