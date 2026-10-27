#pragma once

#include <cstddef>
#include <cstdint>

#include "driver/gpio.h"
#include "driver/uart.h"

namespace config {

// CAN
inline constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_5;
inline constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_4;
inline constexpr uint32_t   CAN_BITRATE = 1000000;
inline constexpr uint32_t   MOTOR_COMMAND = 3;
inline constexpr int64_t    CAN_SEND_FAILURE_TIMEOUT_US = 1000000;

// SBUS
inline constexpr uart_port_t SBUS_UART = UART_NUM_1;
inline constexpr gpio_num_t  SBUS_RX_PIN = GPIO_NUM_6;
inline constexpr uint8_t     SBUS_MODE_CHANNEL = 5;
inline constexpr uint8_t     SBUS_THROTTLE_CHANNEL = 2;
inline constexpr uint8_t     SBUS_STEERING_CHANNEL = 0;
inline constexpr uint8_t     SBUS_HEADER = 0x0F;
inline constexpr int         SBUS_BAUD = 100000;
inline constexpr size_t      SBUS_FRAME_LEN = 25;
inline constexpr uint16_t    SBUS_CHANNEL_MIN = 172;
inline constexpr uint16_t    SBUS_CHANNEL_MID = 992;
inline constexpr uint16_t    SBUS_CHANNEL_MAX = 1811;
inline constexpr size_t      SBUS_UART_RX_BUFFER = 256;

// NUC command input
inline constexpr uart_port_t NUC_UART = UART_NUM_0;
inline constexpr gpio_num_t  NUC_TX_PIN = GPIO_NUM_17;
inline constexpr gpio_num_t  NUC_RX_PIN = GPIO_NUM_18;
inline constexpr int         NUC_BAUDRATE = 115200;
inline constexpr uint8_t     NUC_START_BYTE = 0xA5;
inline constexpr size_t      NUC_PACKET_SIZE = 10;

// Drive and motor configuration
inline constexpr uint8_t MOTOR_FRONT_LEFT = 1;
inline constexpr uint8_t MOTOR_FRONT_RIGHT = 2;
inline constexpr uint8_t MOTOR_REAR_LEFT = 3;
inline constexpr uint8_t MOTOR_REAR_RIGHT = 4;
inline constexpr int     POLE_PAIRS = 21;
inline constexpr int     GEAR_RATIO = 9;
inline constexpr int32_t MAX_MOTOR_ERPM = 200;
inline constexpr double  TWO_PI = 6.28318530717958647692;
inline constexpr int64_t CONTROLLER_INPUT_TIMEOUT_US = 200000;

// Tattu BMS
inline constexpr uint32_t TATTU_BMS_CAN_ID = 0x01109216;
inline constexpr size_t   TATTU_BMS_PAYLOAD_SIZE = 50;

}  // namespace config
