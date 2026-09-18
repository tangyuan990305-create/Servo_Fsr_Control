#ifndef UART_RECEIVER_H
#define UART_RECEIVER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

#define UART_RECEIVER_LINE_MAX_LENGTH  64

typedef struct {
    char text[UART_RECEIVER_LINE_MAX_LENGTH];
} uart_line_t;

/**
 * @brief Initialize UART0 and start the line-receive task.
 *
 * Complete CR/LF terminated lines are placed in a newly created queue.
 *
 * @param[out] line_queue Queue used to deliver uart_line_t items.
 */
esp_err_t uart_receiver_start(QueueHandle_t *line_queue);

#ifdef __cplusplus
}
#endif

#endif
