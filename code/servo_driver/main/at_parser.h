#ifndef AT_PARSER_H
#define AT_PARSER_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef esp_err_t (*at_set_id_handler_t)(int new_id);
typedef esp_err_t (*at_action_handler_t)(
    int servo_id,
    float angle_degrees);
typedef esp_err_t (*at_read_id_handler_t)(void);

/**
 * @brief Start a task that parses AT commands received from a line queue.
 *
 * AT+SEED=<ID> sets the connected servo's ID.
 * AT+ACTION=<servo ID>,<angle> moves a servo to an absolute angle.
 * AT+READID scans the bus and reads each detected servo's EEPROM ID.
 * AT+READCURRENT=<ID> reads the signed present current register.
 * AT+READPOSITION=<ID> reads the present position register.
 */
esp_err_t at_parser_start(
    QueueHandle_t line_queue,
    at_set_id_handler_t set_id_handler,
    at_action_handler_t action_handler,
    at_read_id_handler_t read_id_handler);

#ifdef __cplusplus
}
#endif

#endif
