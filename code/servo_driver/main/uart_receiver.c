#include "uart_receiver.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"

#define UART_RECEIVER_PORT          UART_NUM_0
#define UART_RECEIVER_BAUD_RATE     115200
#define UART_RECEIVER_QUEUE_LENGTH  8
#define UART_RECEIVER_TASK_STACK    3072
#define UART_RECEIVER_TASK_PRIORITY 6
//2026.10.10 修改：输入链路已确认，关闭逐字节调试，避免干扰压力标定日志
//2026.10.12 修改：逐字节日志仅用于早期串口输入排障，
//正常AT测试时关闭，避免大量RX_BYTE日志干扰命令观察
#define UART_RECEIVER_DEBUG_BYTES   0

static const char *TAG = "uart_receiver";
static QueueHandle_t s_line_queue;

static void queue_received_line(uart_line_t *line, size_t *length)
{
    if (*length == 0) {
        return;
    }

    line->text[*length] = '\0';
    ESP_LOGI(TAG, "Received: \"%s\"", line->text);
    if (xQueueSend(
            s_line_queue,
            line,
            pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Line queue full, command discarded");
    }
    *length = 0;
}

static void uart_receive_task(void *argument)
{
    (void)argument;
    uart_line_t line = {0};
    size_t length = 0;
    bool discard_until_newline = false;

    while (true) {
        uint8_t character;
        int received = uart_read_bytes(
            UART_RECEIVER_PORT,
            &character,
            1,
            pdMS_TO_TICKS(100));
        if (received <= 0) {
            //2026.10.10 修改：原实现把100ms空闲误判为命令结束，
            //逐字符输入时 A、T、+ 会被拆成多条命令。
            //现在只接受 CR 或 LF 作为命令结束符。
            continue;
        }

        if (UART_RECEIVER_DEBUG_BYTES) {
            ESP_LOGI(
                TAG,
                "RX_BYTE: 0x%02X",
                (unsigned int)character);
        }

        if (character == '\n') {
            if (!discard_until_newline && length > 0) {
                queue_received_line(&line, &length);
            }

            length = 0;
            discard_until_newline = false;
            continue;
        }

        if (character == '\r') {
            if (!discard_until_newline) {
                queue_received_line(&line, &length);
            }
            continue;
        }

        if (discard_until_newline) {
            continue;
        }

        if (length < sizeof(line.text) - 1) {
            line.text[length++] = (char)character;
        } else {
            ESP_LOGW(TAG, "Input line too long, command discarded");
            length = 0;
            discard_until_newline = true;
        }
    }
}

esp_err_t uart_receiver_start(QueueHandle_t *line_queue)
{
    if (line_queue == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_line_queue != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_line_queue = xQueueCreate(
        UART_RECEIVER_QUEUE_LENGTH,
        sizeof(uart_line_t));
    if (s_line_queue == NULL) {
        return ESP_ERR_NO_MEM;
    }

    const uart_config_t config = {
        .baud_rate = UART_RECEIVER_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_DEFAULT,
    };

    esp_err_t error = uart_param_config(UART_RECEIVER_PORT, &config);
    if (error != ESP_OK) {
        vQueueDelete(s_line_queue);
        s_line_queue = NULL;
        return error;
    }

    error = uart_driver_install(
        UART_RECEIVER_PORT,
        256,
        0,
        0,
        NULL,
        0);
    if (error != ESP_OK) {
        vQueueDelete(s_line_queue);
        s_line_queue = NULL;
        return error;
    }

    if (xTaskCreate(
            uart_receive_task,
            "uart_receive",
            UART_RECEIVER_TASK_STACK,
            NULL,
            UART_RECEIVER_TASK_PRIORITY,
            NULL) != pdPASS) {
        uart_driver_delete(UART_RECEIVER_PORT);
        vQueueDelete(s_line_queue);
        s_line_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    *line_queue = s_line_queue;
    ESP_LOGI(TAG, "UART0 receiver ready at %d baud", UART_RECEIVER_BAUD_RATE);
    return ESP_OK;
}
