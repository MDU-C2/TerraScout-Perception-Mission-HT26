#include "can.hpp"
#include "motor_interface.hpp"
#include "sbus.hpp"
#include "uart.hpp"

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include <cstdint>
#include <cstring>

namespace {

constexpr char TAG[] = "MAIN";

constexpr int NUM_MOTORS = 4;
constexpr int CONTROL_PERIOD_MS = 10;
constexpr int64_t SBUS_TIMEOUT_US = 100000;
constexpr int64_t UART_TIMEOUT_US = 200000;

constexpr uart_port_t SBUS_UART_NUM = UART_NUM_1;
constexpr gpio_num_t SBUS_RX_PIN = GPIO_NUM_25;
constexpr uart_port_t PC_UART_NUM = UART_NUM_0;
constexpr gpio_num_t PC_TX_PIN = GPIO_NUM_17;
constexpr gpio_num_t PC_RX_PIN = GPIO_NUM_16;
constexpr int PC_BAUD = 115200;
constexpr gpio_num_t CAN_TX_PIN = GPIO_NUM_5;
constexpr gpio_num_t CAN_RX_PIN = GPIO_NUM_4;

constexpr int CH_STEER = 0;
constexpr int CH_THROTTLE = 2;
constexpr int CH_MODE_SWITCH = 4;

constexpr int32_t SBUS_MIN = 172;
constexpr int32_t SBUS_MID = 992;
constexpr int32_t SBUS_MAX = 1811;
constexpr int32_t SBUS_DEADBAND = 20;
constexpr int32_t MAX_RPM = 1000;

constexpr uint8_t UART_HEADER_0 = 0xAA;
constexpr uint8_t UART_HEADER_1 = 0x55;
constexpr size_t UART_PAYLOAD_LEN = NUM_MOTORS * sizeof(int16_t);
constexpr size_t UART_PACKET_LEN = 2 + UART_PAYLOAD_LEN + 1;
constexpr uint8_t MOTOR_IDS[NUM_MOTORS] = {1, 2, 3, 4};

enum class DriveMode {
    Disabled,
    Manual,
    Autonomous,
};

struct SharedInputs {
    SbusFrame sbus{};
    int64_t sbus_received_us = 0;
    int16_t uart_rpm[NUM_MOTORS]{};
    int64_t uart_received_us = 0;
};

struct TaskContext {
    Sbus* sbus;
    Uart* uart;
    Can* can;
    SemaphoreHandle_t mutex;
    SharedInputs inputs;
};

int16_t clampRpm(int32_t rpm)
{
    if (rpm > MAX_RPM)
        return MAX_RPM;
    if (rpm < -MAX_RPM)
        return -MAX_RPM;
    return static_cast<int16_t>(rpm);
}

int16_t sbusToRpm(uint16_t raw)
{
    const int32_t offset = static_cast<int32_t>(raw) - SBUS_MID;
    if (offset > -SBUS_DEADBAND && offset < SBUS_DEADBAND)
        return 0;

    const int32_t span = offset > 0 ? SBUS_MAX - SBUS_MID : SBUS_MID - SBUS_MIN;
    return clampRpm(offset * MAX_RPM / span);
}

DriveMode modeFromSwitch(uint16_t raw)
{
    if (raw < 600)
        return DriveMode::Disabled;
    if (raw < 1400)
        return DriveMode::Manual;
    return DriveMode::Autonomous;
}

const char* modeName(DriveMode mode)
{
    switch (mode) {
    case DriveMode::Manual:
        return "MANUAL";
    case DriveMode::Autonomous:
        return "AUTONOMOUS";
    case DriveMode::Disabled:
    default:
        return "DISABLED";
    }
}

void sbusTask(void* arg)
{
    auto* context = static_cast<TaskContext*>(arg);
    SbusFrame frame{};

    while (true) {
        if (context->sbus->read(frame, pdMS_TO_TICKS(20)) && frame.valid) {
            xSemaphoreTake(context->mutex, portMAX_DELAY);
            context->inputs.sbus = frame;
            context->inputs.sbus_received_us = esp_timer_get_time();
            xSemaphoreGive(context->mutex);
        }
    }
}

bool parseUartPacket(const uint8_t* packet, int16_t (&rpm)[NUM_MOTORS])
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < UART_PAYLOAD_LEN; ++i)
        checksum ^= packet[2 + i];

    if (checksum != packet[UART_PACKET_LEN - 1])
        return false;

    for (int motor = 0; motor < NUM_MOTORS; ++motor) {
        const size_t offset = 2 + motor * sizeof(int16_t);
        int32_t value = static_cast<int32_t>(packet[offset]) |
                        (static_cast<int32_t>(packet[offset + 1]) << 8);
        if (value & 0x8000)
            value -= 0x10000;
        rpm[motor] = clampRpm(value);
    }
    return true;
}

void uartTask(void* arg)
{
    auto* context = static_cast<TaskContext*>(arg);
    uint8_t rx[64];
    uint8_t packet[UART_PACKET_LEN]{};
    size_t packet_size = 0;

    while (true) {
        const int bytes_read = context->uart->receive(
            rx, sizeof(rx), pdMS_TO_TICKS(20));
        if (bytes_read <= 0)
            continue;

        for (int i = 0; i < bytes_read; ++i) {
            const uint8_t byte = rx[i];
            if (packet_size == 0) {
                if (byte == UART_HEADER_0)
                    packet[packet_size++] = byte;
                continue;
            }
            if (packet_size == 1 && byte != UART_HEADER_1) {
                packet_size = 0;
                if (byte == UART_HEADER_0)
                    packet[packet_size++] = byte;
                continue;
            }

            packet[packet_size++] = byte;
            if (packet_size == UART_PACKET_LEN) {
                int16_t rpm[NUM_MOTORS]{};
                if (parseUartPacket(packet, rpm)) {
                    xSemaphoreTake(context->mutex, portMAX_DELAY);
                    std::memcpy(context->inputs.uart_rpm, rpm, sizeof(rpm));
                    context->inputs.uart_received_us = esp_timer_get_time();
                    xSemaphoreGive(context->mutex);
                } else {
                    ESP_LOGW(TAG, "UART packet checksum error");
                }
                packet_size = 0;
            }
        }
    }
}

bool sendMotorRpm(Can& can, uint8_t motor_id, int16_t rpm)
{
    const twai_message_t message = cubemars::encodeRpm(motor_id, rpm);
    return can.send(message.identifier, message.data,
                    message.data_length_code, message.extd, message.rtr,
                    pdMS_TO_TICKS(5));
}

void controlTask(void* arg)
{
    auto* context = static_cast<TaskContext*>(arg);
    TickType_t last_wake = xTaskGetTickCount();
    DriveMode previous_mode = DriveMode::Disabled;
    int log_divider = 0;

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        SharedInputs inputs;
        xSemaphoreTake(context->mutex, portMAX_DELAY);
        inputs = context->inputs;
        xSemaphoreGive(context->mutex);

        const int64_t now = esp_timer_get_time();
        const bool sbus_valid =
            inputs.sbus_received_us != 0 &&
            now - inputs.sbus_received_us < SBUS_TIMEOUT_US &&
            inputs.sbus.valid && !inputs.sbus.failsafe &&
            !inputs.sbus.frame_lost;
        const DriveMode mode = sbus_valid
            ? modeFromSwitch(inputs.sbus.channels[CH_MODE_SWITCH])
            : DriveMode::Disabled;

        int16_t rpm[NUM_MOTORS]{};
        if (mode == DriveMode::Manual) {
            const int16_t throttle = sbusToRpm(inputs.sbus.channels[CH_THROTTLE]);
            const int16_t steer = sbusToRpm(inputs.sbus.channels[CH_STEER]);
            const int16_t left = clampRpm(static_cast<int32_t>(throttle) + steer);
            const int16_t right = clampRpm(static_cast<int32_t>(throttle) - steer);
            rpm[0] = left;
            rpm[2] = left;
            rpm[1] = right;
            rpm[3] = right;
        } else if (mode == DriveMode::Autonomous &&
                   inputs.uart_received_us != 0 &&
                   now - inputs.uart_received_us < UART_TIMEOUT_US) {
            std::memcpy(rpm, inputs.uart_rpm, sizeof(rpm));
        }

        if (mode != previous_mode) {
            ESP_LOGI(TAG, "Mode: %s -> %s",
                     modeName(previous_mode), modeName(mode));
            previous_mode = mode;
        }

        for (int motor = 0; motor < NUM_MOTORS; ++motor) {
            if (!sendMotorRpm(*context->can, MOTOR_IDS[motor], rpm[motor]) &&
                log_divider == 0) {
                ESP_LOGW(TAG, "CAN send failed for motor %u",
                         static_cast<unsigned>(MOTOR_IDS[motor]));
            }
        }

        if (++log_divider >= 100) {
            log_divider = 0;
            ESP_LOGI(TAG, "%s RPM=[%d %d %d %d]", modeName(mode),
                     rpm[0], rpm[1], rpm[2], rpm[3]);
        }
    }
}

} // namespace

extern "C" void app_main(void)
{
    static Sbus sbus(SBUS_UART_NUM, SBUS_RX_PIN);
    static Uart pc_uart(PC_UART_NUM, PC_TX_PIN, PC_RX_PIN, PC_BAUD);
    static Can can(CAN_TX_PIN, CAN_RX_PIN);
    static TaskContext context{
        &sbus,
        &pc_uart,
        &can,
        xSemaphoreCreateMutex(),
        {},
    };

    configASSERT(context.mutex != nullptr);
    configASSERT(xTaskCreate(sbusTask, "SBUS", 3072, &context, 6, nullptr) == pdPASS);
    configASSERT(xTaskCreate(uartTask, "UART", 3072, &context, 5, nullptr) == pdPASS);
    configASSERT(xTaskCreate(controlTask, "CONTROL", 4096, &context, 7, nullptr) == pdPASS);

    ESP_LOGI(TAG, "Motor control tasks started");
}