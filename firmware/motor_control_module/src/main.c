#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"

#include "uart.h"
#include "sbus.h"
#include "can.h"

static const char *TAG = "MAIN";

#define NUM_MOTORS 4
#define CONTROL_PERIOD_MS 10
#define SBUS_TIMEOUT_US 100000
#define PC_TIMEOUT_US 200000

#define SBUS_UART_NUM UART_NUM_1
#define SBUS_RX_PIN GPIO_NUM_25
#define PC_UART_NUM UART_NUM_0
#define PC_TX_PIN GPIO_NUM_17
#define PC_RX_PIN GPIO_NUM_16
#define PC_BAUD 115200
#define CAN_TX_PIN GPIO_NUM_5
#define CAN_RX_PIN GPIO_NUM_4

#define CH_STEER 0
#define CH_THROTTLE 2
#define CH_MODE_SWITCH 4

#define SBUS_MIN 172
#define SBUS_MID 992
#define SBUS_MAX 1811
#define SBUS_DEADBAND 20

#define CMD_MAX 1000

#define CAN_MOTOR_BASE_ID 0x100

#define PC_HDR0 0xAA
#define PC_HDR1 0x55
#define PC_PAYLOAD_LEN (NUM_MOTORS * 2)
#define PC_PACKET_LEN (2 + PC_PAYLOAD_LEN + 1)

typedef enum
{
    MODE_DISABLED = 0,
    MODE_MANUAL,
    MODE_AUTONOMOUS,
} drive_mode_t;

static const char *mode_name(drive_mode_t m)
{
    switch (m)
    {
    case MODE_MANUAL:
        return "MANUAL";
    case MODE_AUTONOMOUS:
        return "AUTONOMOUS";
    default:
        return "DISABLED";
    }
}

typedef struct
{
    sbus_data_t sbus;
    int64_t sbus_valid_us;
    int16_t pc_cmd[NUM_MOTORS];
    int64_t pc_valid_us;
} shared_t;

static shared_t s_shared;
static SemaphoreHandle_t s_lock;

static sbus_driver_handle_t s_sbus;
static uart_driver_handle_t s_pc_uart;
static can_driver_handle_t s_can;

static int16_t clamp_cmd(int32_t v)
{
    if (v > CMD_MAX)
        return CMD_MAX;
    if (v < -CMD_MAX)
        return -CMD_MAX;
    return (int16_t)v;
}

static int16_t sbus_to_cmd(uint16_t raw)
{
    int32_t v = (int32_t)raw - SBUS_MID;
    if (v > -SBUS_DEADBAND && v < SBUS_DEADBAND)
        return 0;
    int32_t span = (v > 0) ? (SBUS_MAX - SBUS_MID) : (SBUS_MID - SBUS_MIN);
    return clamp_cmd(v * CMD_MAX / span);
}

static drive_mode_t switch_to_mode(uint16_t raw)
{
    if (raw < 600)
        return MODE_DISABLED; /* ~172  */
    if (raw < 1400)
        return MODE_MANUAL; /* ~992  */
    return MODE_AUTONOMOUS; /* ~1811 */
}

static esp_err_t send_motor(int idx, int16_t cmd, bool enable)
{
    can_frame_t f = {
        .id = CAN_MOTOR_BASE_ID + idx,
        .is_extended = false,
        .is_rtr = false,
        .dlc = 3,
    };
    f.data[0] = (uint8_t)(cmd & 0xFF);
    f.data[1] = (uint8_t)((cmd >> 8) & 0xFF);
    f.data[2] = enable ? 1 : 0;
    return can_driver_send(s_can, &f, pdMS_TO_TICKS(5));
}

static void sbus_task(void *arg)
{
    sbus_data_t d;
    while (1)
    {
        esp_err_t r = sbus_driver_read(s_sbus, &d, 20);
        if (r == ESP_OK)
        {
            xSemaphoreTake(s_lock, portMAX_DELAY);
            s_shared.sbus = d;
            s_shared.sbus_valid_us = esp_timer_get_time();
            xSemaphoreGive(s_lock);
        }
    }
}

static void pc_task(void *arg)
{
    uint8_t rx[64];
    uint8_t pkt[PC_PACKET_LEN];
    int idx = 0;

    while (1)
    {
        size_t n = 0;
        if (uart_driver_read(s_pc_uart, rx, sizeof(rx), &n, pdMS_TO_TICKS(20)) != ESP_OK || n == 0)
        {
            continue;
        }

        for (size_t i = 0; i < n; i++)
        {
            uint8_t b = rx[i];

            if (idx == 0 && b != PC_HDR0)
                continue;
            if (idx == 1 && b != PC_HDR1)
            {
                idx = (b == PC_HDR0) ? 1 : 0;
                if (idx == 1)
                    pkt[0] = PC_HDR0;
                continue;
            }
            pkt[idx++] = b;

            if (idx == PC_PACKET_LEN)
            {
                uint8_t x = 0;
                for (int k = 0; k < PC_PAYLOAD_LEN; k++)
                    x ^= pkt[2 + k];

                if (x == pkt[PC_PACKET_LEN - 1])
                {
                    int16_t cmd[NUM_MOTORS];
                    for (int m = 0; m < NUM_MOTORS; m++)
                    {
                        cmd[m] = clamp_cmd((int16_t)(pkt[2 + m * 2] | (pkt[3 + m * 2] << 8)));
                    }
                    xSemaphoreTake(s_lock, portMAX_DELAY);
                    memcpy(s_shared.pc_cmd, cmd, sizeof(cmd));
                    s_shared.pc_valid_us = esp_timer_get_time();
                    xSemaphoreGive(s_lock);
                }
                else
                {
                    ESP_LOGW(TAG, "PC packet checksum error");
                }
                idx = 0;
            }
        }
    }
}

static void control_task(void *arg)
{
    TickType_t last = xTaskGetTickCount();
    drive_mode_t prev_mode = MODE_DISABLED;
    int log_div = 0;

    while (1)
    {
        vTaskDelayUntil(&last, pdMS_TO_TICKS(CONTROL_PERIOD_MS));

        shared_t s;
        xSemaphoreTake(s_lock, portMAX_DELAY);
        s = s_shared;
        xSemaphoreGive(s_lock);

        int64_t now = esp_timer_get_time();

        bool rc_ok = s.sbus_valid_us != 0 &&
                     (now - s.sbus_valid_us) < SBUS_TIMEOUT_US &&
                     !s.sbus.failsafe && !s.sbus.frame_lost;

        drive_mode_t mode = rc_ok ? switch_to_mode(s.sbus.channels[CH_MODE_SWITCH])
                                  : MODE_DISABLED;

        int16_t cmd[NUM_MOTORS] = {0};
        bool enable = false;

        switch (mode)
        {
        case MODE_MANUAL:
        {
            int16_t thr = sbus_to_cmd(s.sbus.channels[CH_THROTTLE]);
            int16_t str = sbus_to_cmd(s.sbus.channels[CH_STEER]);
            int16_t left = clamp_cmd((int32_t)thr + str);
            int16_t right = clamp_cmd((int32_t)thr - str);
            cmd[0] = left;
            cmd[2] = left;
            cmd[1] = right;
            cmd[3] = right;
            enable = true;
            break;
        }
        case MODE_AUTONOMOUS:
        {
            bool pc_ok = s.pc_valid_us != 0 && (now - s.pc_valid_us) < PC_TIMEOUT_US;
            if (pc_ok)
            {
                memcpy(cmd, s.pc_cmd, sizeof(cmd));
            }
            enable = true;
            break;
        }
        case MODE_DISABLED:
        default:
            break;
        }

        if (mode != prev_mode)
        {
            ESP_LOGI(TAG, "Mode: %s -> %s", mode_name(prev_mode), mode_name(mode));
            prev_mode = mode;
        }

        for (int m = 0; m < NUM_MOTORS; m++)
        {
            esp_err_t e = send_motor(m, cmd[m], enable);
            if (e != ESP_OK && (log_div % 100) == 0)
            {
                ESP_LOGW(TAG, "CAN send motor %d failed: %s", m, esp_err_to_name(e));
            }
        }

        if (++log_div >= 100)
        { /* ~1 Hz status */
            log_div = 0;
            ESP_LOGI(TAG, "%s  cmd=[%d %d %d %d]", mode_name(mode),
                     cmd[0], cmd[1], cmd[2], cmd[3]);
        }
    }
}

void app_main(void)
{
    s_lock = xSemaphoreCreateMutex();
    configASSERT(s_lock);

    sbus_driver_config_t sbus_cfg = {
        .uart_cfg = {
            .uart_num = SBUS_UART_NUM,
            .tx_pin = UART_PIN_NO_CHANGE,
            .rx_pin = SBUS_RX_PIN,
            .rx_buffer_size = 512,
            .tx_buffer_size = 0,
        }};
    ESP_ERROR_CHECK(sbus_driver_init(&sbus_cfg, &s_sbus));

    /* PC link */
    uart_driver_config_t pc_cfg = {
        .uart_num = PC_UART_NUM,
        .tx_pin = PC_TX_PIN,
        .rx_pin = PC_RX_PIN,
        .baud_rate = PC_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_buffer_size = 1024,
        .tx_buffer_size = 0,
    };
    ESP_ERROR_CHECK(uart_driver_init(&pc_cfg, &s_pc_uart));

    /* CAN */
    can_driver_config_t can_cfg = {
        .tx_pin = CAN_TX_PIN,
        .rx_pin = CAN_RX_PIN,
        .timing = TWAI_TIMING_CONFIG_500KBITS(),
        .filter = TWAI_FILTER_CONFIG_ACCEPT_ALL(),
    };
    ESP_ERROR_CHECK(can_driver_init(&can_cfg, &s_can));
    ESP_ERROR_CHECK(can_driver_start(s_can));

    xTaskCreate(sbus_task, "SBUS", 3072, NULL, 6, NULL);
    xTaskCreate(pc_task, "NUC_UART", 3072, NULL, 5, NULL);
    xTaskCreate(control_task, "CONTROL", 4096, NULL, 7, NULL);

    ESP_LOGI(TAG, "Started");
}