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
            /*
             * Some serial terminals do not append CR/LF in text mode.
             * Treat 100 ms of idle time as the end of a non-empty command.
             */
            if (!discard_until_newline) {
                queue_received_line(&line, &length);
            } else {
                discard_until_newline = false;
            }
            continue;
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
