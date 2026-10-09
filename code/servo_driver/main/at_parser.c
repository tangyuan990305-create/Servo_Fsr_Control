#include "at_parser.h"

#include <errno.h>
#include <limits.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/task.h"
#include "fsr_monitor.h"
#include "servo_task.h"
#include "system_commands.h"
#include "uart_receiver.h"

#define AT_PARSER_TASK_STACK     3072
#define AT_PARSER_TASK_PRIORITY  5
#define BUTTON_PRESS_HOLD_MS     1500
#define BUTTON_RETURN_SETTLE_MS  1000
#define BUTTON_DOUBLE_GAP_MS     1500
#define BUTTON_LONG_EXTRA_MS     500
#define POWER_BUTTON_HOLD_MS     9500
#define POWER_ON_BUTTON_HOLD_MS  3800
#define POWER_OFF_STAGE_DELAY_MS  250
#define AI_BUTTON_SERVO_ID       2
#define FSR_ACTION_CAPTURE_MS    2000
#define FSR_REPEAT_EXTRA_SETTLE_MS 1500

static const char *TAG = "at_parser";
static QueueHandle_t s_line_queue;
static at_set_id_handler_t s_set_id_handler;
static at_action_handler_t s_action_handler;
static at_read_id_handler_t s_read_id_handler;
static bool s_started;
static unsigned int s_force_sample_count;
static float s_force_sum_g;
static float s_force_average_g;
static float s_force_min_g;
static float s_force_max_g;
static float s_force_last_g;

static void fsr_log_peak(const char *action);

static void send_run_over(void)
{
    const char response[] = "run over\r\n";
    uart_write_bytes(
        UART_NUM_0,
        response,
        sizeof(response) - 1);
}

static void fsr_stats_reset(void)
{
    s_force_sample_count = 0;
    s_force_sum_g = 0.0f;
    s_force_average_g = 0.0f;
    s_force_min_g = 0.0f;
    s_force_max_g = 0.0f;
    s_force_last_g = 0.0f;
}

static esp_err_t run_fsr_repeat_test(
    int servo_id,
    int count)
{
    if ((servo_id != 1 && servo_id != 2) ||
        count < 1 || count > 50) {
        return ESP_ERR_INVALID_ARG;
    }

    fsr_stats_reset();
    fsr_monitor_set_periodic_logging(false);

    esp_err_t error = ESP_OK;
    for (int index = 0; index < count; ++index) {
        fsr_monitor_reset_peak();
        ESP_LOGI(
            TAG,
            "FSR repeat test: %d/%d",
            index + 1,
            count);

        error = s_action_handler(
            servo_id,
            servo_get_press_angle(servo_id));
        if (error != ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_PRESS_HOLD_MS));

        error = s_action_handler(servo_id, 0);
        if (error != ESP_OK) {
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(BUTTON_RETURN_SETTLE_MS));

        fsr_log_peak("Repeat test");
        if (index + 1 < count) {
            vTaskDelay(pdMS_TO_TICKS(FSR_REPEAT_EXTRA_SETTLE_MS));
        }
    }

    return error;
}

static void fsr_log_peak(const char *action)
{
    const float peak_g = fsr_monitor_get_peak_grams();

    ++s_force_sample_count;
    s_force_sum_g += peak_g;
    s_force_last_g = peak_g;
    s_force_average_g =
        s_force_sum_g / (float)s_force_sample_count;

    if (s_force_sample_count == 1 || peak_g < s_force_min_g) {
        s_force_min_g = peak_g;
    }
    if (s_force_sample_count == 1 || peak_g > s_force_max_g) {
        s_force_max_g = peak_g;
    }

    ESP_LOGI(
        TAG,
        "%s FSR peak: %dmV, %.0fg, %u ms; "
        "samples=%u avg=%.0fg min=%.0fg max=%.0fg",
        action,
        fsr_monitor_get_peak_mv(),
        peak_g,
        fsr_monitor_get_peak_time_ms(),
        s_force_sample_count,
        s_force_average_g,
        s_force_min_g,
        s_force_max_g);
}

static bool parse_integer(
    const char *text,
    char **end,
    int *value_out)
{
    errno = 0;
    char *number_end = NULL;
    long value = strtol(text, &number_end, 10);
    if (errno == ERANGE || value < INT_MIN || value > INT_MAX ||
        number_end == text) {
        return false;
    }

    *end = number_end;
    *value_out = (int)value;
    return true;
}

static bool parse_integer_parameter(
    const char *line,
    const char *prefix,
    int *value_out)
{
    size_t prefix_length = strlen(prefix);
    const char *number = line + prefix_length;

    if (strncmp(line, prefix, prefix_length) != 0 ||
        *number == '\0') {
        return false;
    }

    char *end = NULL;
    if (!parse_integer(number, &end, value_out) || *end != '\0') {
        return false;
    }

    return true;
}

static bool parse_id_command(const char *line, const char *prefix, int *id)
{
    return parse_integer_parameter(line, prefix, id);
}

static bool parse_two_integer_parameters(
    const char *line,
    const char *prefix,
    int *first,
    float *second)
{
    size_t prefix_length = strlen(prefix);
    const char *parameter = line + prefix_length;

    if (strncmp(line, prefix, prefix_length) != 0) {
        return false;
    }

    char *end = NULL;
    if (!parse_integer(parameter, &end, first) || *end != ',') {
        return false;
    }

    errno = 0;
    char *angle_end = NULL;
    float angle = strtof(end + 1, &angle_end);
    if (errno == ERANGE || angle_end == end + 1 || *angle_end != '\0') {
        return false;
    }

    float tenth_steps = angle * 10.0f;  //之前是2.0f，变量名由half_steps改为tenth_steps，表示角度的十分之一
    int rounded_tenth_steps = tenth_steps >= 0.0f
        ? (int)(tenth_steps + 0.5f)
        : (int)(tenth_steps - 0.5f);
    float difference = tenth_steps - (float)rounded_tenth_steps;
    if (difference < 0.0f) {
        difference = -difference;
    }
    if (difference > 0.001f) {
        return false;
    }

    *second = angle;
    return true;
}

static bool is_power_button_command(const char *line)
{
    return strcmp(line, "AT+LPOWEROFF?") == 0;
}

static bool is_power_on_command(const char *line)
{
    return strcmp(line, "AT+LPOWERON?") == 0;
}

static esp_err_t run_power_button_action(void)
{
    fsr_monitor_reset_peak();
    esp_err_t error = s_action_handler(1, servo_get_press_angle(1));
    if (error == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(POWER_OFF_STAGE_DELAY_MS));
        error = s_action_handler(2, servo_get_press_angle(2));
    }
    if (error == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(POWER_BUTTON_HOLD_MS));
        error = s_action_handler(1, 0);
    }
    if (error == ESP_OK) {
        error = s_action_handler(2, 0);
    }
    if (error == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_RETURN_SETTLE_MS));
    }
    return error;
}

static esp_err_t run_power_on_action(void)
{
    fsr_monitor_reset_peak();
    esp_err_t error = s_action_handler(1, servo_get_press_angle(1));
    if (error == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(POWER_ON_BUTTON_HOLD_MS));
        error = s_action_handler(1, 0);
    }
    if (error == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_RETURN_SETTLE_MS));
    }
    return error;
}

static esp_err_t run_single_servo_button_action(
    int servo_id,
    int hold_ms)
{
    fsr_monitor_reset_peak();
    esp_err_t error =
        s_action_handler(servo_id, servo_get_press_angle(servo_id));
    if (error == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(hold_ms));
        error = s_action_handler(servo_id, 0);
    }
    if (error == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(BUTTON_RETURN_SETTLE_MS));
    }
    return error;
}

static void at_parse_task(void *argument)
{
    (void)argument;
    uart_line_t line;

    while (true) {
        if (xQueueReceive(s_line_queue, &line, portMAX_DELAY) != pdTRUE) {
            continue;
        }

        int servo_id;
        float press_angle;
        int value;
        float angle;
        esp_err_t error;
        if (system_command_handle(line.text)) {
            continue;
        } else if (parse_two_integer_parameters(
                       line.text,
                       "AT+FSRTEST=",
                       &servo_id,
                       &angle)) {
            int repeat_count = (int)angle;
            if ((float)repeat_count != angle ||
                repeat_count < 1 || repeat_count > 50) {
                error = ESP_ERR_INVALID_ARG;
            } else {
                error = run_fsr_repeat_test(
                    servo_id,
                    repeat_count);
            }

            if (error == ESP_OK) {
                char response[160];
                snprintf(
                    response,
                    sizeof(response),
                    "+FSRTEST:servo=%d,count=%u,avg=%.0f,"
                    "min=%.0f,max=%.0f,last=%.0f\r\nOK\r\n",
                    servo_id,
                    s_force_sample_count,
                    s_force_average_g,
                    s_force_min_g,
                    s_force_max_g,
                    s_force_last_g);
                uart_write_bytes(UART_NUM_0, response, strlen(response));
            } else {
                char response[96];
                snprintf(
                    response,
                    sizeof(response),
                    "+FSRTEST:ERROR,completed=%u\r\n",
                    s_force_sample_count);
                uart_write_bytes(UART_NUM_0, response, strlen(response));
            }
            send_run_over();
            ESP_ERROR_CHECK(uart_wait_tx_done(
                UART_NUM_0,
                pdMS_TO_TICKS(100)));
            fsr_monitor_set_periodic_logging(true);
        } else if (strcmp(line.text, "AT+FSR?") == 0) {
            char response[80];
            snprintf(
                response,
                sizeof(response),
                "+FSR:%d,%+.0f,zero=%d\r\nOK\r\n",
                fsr_monitor_get_current_mv(),
                fsr_monitor_get_current_grams(),
                fsr_monitor_get_zero_mv());
            uart_write_bytes(UART_NUM_0, response, strlen(response));
            send_run_over();
        } else if (strcmp(line.text, "AT+FSRZERO") == 0) {
            error = fsr_monitor_capture_zero();
            if (error == ESP_OK) {
                char response[64];
                snprintf(
                    response,
                    sizeof(response),
                    "+FSRZERO:%d\r\nOK\r\n",
                    fsr_monitor_get_zero_mv());
                uart_write_bytes(UART_NUM_0, response, strlen(response));
            } else {
                const char response[] = "+FSRZERO:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
            }
            send_run_over();
        } else if (strcmp(line.text, "AT+FSRRESET") == 0) {
            fsr_monitor_reset_peak();
            const char response[] = "+FSRRESET:OK\r\n";
            uart_write_bytes(
                UART_NUM_0,
                response,
                sizeof(response) - 1);
            send_run_over();
        } else if (strcmp(line.text, "AT+FSRPEAK?") == 0) {
            char response[80];
            snprintf(
                response,
                sizeof(response),
                "+FSRPEAK:%d,%+.0f,%ums\r\nOK\r\n",
                fsr_monitor_get_peak_mv(),
                fsr_monitor_get_peak_grams(),
                fsr_monitor_get_peak_time_ms());
            uart_write_bytes(UART_NUM_0, response, strlen(response));
            send_run_over();
        } else if (strcmp(line.text, "AT+FSRSTATS?") == 0) {
            char response[128];
            snprintf(
                response,
                sizeof(response),
                "+FSRSTATS:count=%u,avg=%.0f,min=%.0f,"
                "max=%.0f,last=%.0f\r\nOK\r\n",
                s_force_sample_count,
                s_force_average_g,
                s_force_min_g,
                s_force_max_g,
                s_force_last_g);
            uart_write_bytes(UART_NUM_0, response, strlen(response));
            send_run_over();
        } else if (strcmp(line.text, "AT+FSRSTATSRESET") == 0) {
            fsr_stats_reset();
            const char response[] = "+FSRSTATSRESET:OK\r\n";
            uart_write_bytes(
                UART_NUM_0,
                response,
                sizeof(response) - 1);
            send_run_over();
        } else if (parse_id_command(line.text, "AT+READCURRENT=", &servo_id)) {
            int16_t current = 0;
            error = servo_read_current(servo_id, &current);
            if (error == ESP_OK) {
                char response[64];
                snprintf(response, sizeof(response), "+READCURRENT:%d\r\nOK\r\n", current);
                uart_write_bytes(UART_NUM_0, response, strlen(response));
            } else {
                const char response[] = "+READCURRENT:ERROR\r\n";
                uart_write_bytes(UART_NUM_0, response, sizeof(response) - 1);
            }
            send_run_over();
        } else if (parse_id_command(line.text, "AT+READPOSITION=", &servo_id)) {
            uint16_t position = 0;
            error = servo_read_position(servo_id, &position);
            if (error == ESP_OK) {
                char response[64];
                snprintf(response, sizeof(response), "+READPOSITION:%u\r\nOK\r\n", position);
                uart_write_bytes(UART_NUM_0, response, strlen(response));
            } else {
                const char response[] = "+READPOSITION:ERROR\r\n";
                uart_write_bytes(UART_NUM_0, response, sizeof(response) - 1);
            }
            send_run_over();
        } else if (is_power_button_command(line.text)) {
            error = run_power_button_action();
            if (error == ESP_OK) {
                const char response[] =
                    "OK\r\n+LPOWEROFF:OK\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                fsr_log_peak("Power-off button");
                ESP_LOGI(
                    TAG,
                    "Power-off action completed: servo 1 %+.1f, servo 2 %+.1f",
                    servo_get_press_angle(1),
                    servo_get_press_angle(2));
            } else {
                const char response[] =
                    "+LPOWEROFF:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                ESP_LOGE(
                    TAG,
                    "Power-off action failed: %s",
                    esp_err_to_name(error));
            }
            send_run_over();
        } else if (is_power_on_command(line.text)) {
            error = run_power_on_action();
            if (error == ESP_OK) {
                const char response[] =
                    "OK\r\n+LPOWERON:OK\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                fsr_log_peak("Power-on button");
                ESP_LOGI(
                    TAG,
                    "Power-on action completed: servo 1 %+.1f, %d ms",
                    servo_get_press_angle(1),
                    POWER_ON_BUTTON_HOLD_MS);
            } else {
                const char response[] =
                    "+LPOWERON:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                ESP_LOGE(
                    TAG,
                    "Power-on action failed: %s",
                    esp_err_to_name(error));
            }
            send_run_over();
        } else if (parse_integer_parameter(
                       line.text,
                       "AT+LAILONGBUTTON=",
                       &value)) {
            if (value < 1 || value > 60000) {
                error = ESP_ERR_INVALID_ARG;
            } else {
                error = run_single_servo_button_action(
                    AI_BUTTON_SERVO_ID,
                    value + BUTTON_LONG_EXTRA_MS);
            }

            if (error == ESP_OK) {
                const char response[] =
                    "+LAILONGBUTTON:OK\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                fsr_log_peak("AI long press");
                ESP_LOGI(
                    TAG,
                    "AI button long-press completed: %d ms",
                    value + BUTTON_LONG_EXTRA_MS);
            } else {
                const char response[] =
                    "+LAILONGBUTTON:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                ESP_LOGE(
                    TAG,
                    "AI button long-press failed: time=%d ms, %s",
                    value,
                    esp_err_to_name(error));
            }
            send_run_over();
        } else if (parse_integer_parameter(
                       line.text,
                       "AT+LONGBUTTON=",
                       &value)) {
            if (value < 1 || value > 60000) {
                error = ESP_ERR_INVALID_ARG;
            } else {
                servo_id = 1;
                press_angle = servo_get_press_angle(1);
                fsr_monitor_reset_peak();
                error = s_action_handler(servo_id, press_angle);
                if (error == ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(
                        value + BUTTON_LONG_EXTRA_MS));
                    error = s_action_handler(servo_id, 0);
                    if (error == ESP_OK) {
                        vTaskDelay(pdMS_TO_TICKS(
                            BUTTON_RETURN_SETTLE_MS));
                    }
                }
            }

            if (error == ESP_OK) {
                const char response[] =
                    "+LLONGBUTTON:OK\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                fsr_log_peak("Long press");
                ESP_LOGI(
                    TAG,
                    "Button long-press completed: %d ms",
                    value + BUTTON_LONG_EXTRA_MS);
            } else {
                const char response[] =
                    "+LLONGBUTTON:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                ESP_LOGE(
                    TAG,
                    "Button long-press failed: time=%d ms, %s",
                    value,
                    esp_err_to_name(error));
            }
            send_run_over();
        } else if (parse_integer_parameter(
                line.text,
                "AT+SEED=",
                &value)) {
            error = s_set_id_handler(value);
            if (error == ESP_ERR_INVALID_ARG) {
                ESP_LOGW(
                    TAG,
                    "Invalid servo ID: %d (valid range: 1-253)",
                    value);
            } else if (error != ESP_OK) {
                ESP_LOGE(
                    TAG,
                    "AT+SEED handler failed: %s",
                    esp_err_to_name(error));
            } else {
                ESP_LOGI(TAG, "OK: servo ID set to %d", value);
            }
            send_run_over();
        } else if (strcmp(line.text, "AT+READID") == 0) {
            error = s_read_id_handler();
            if (error == ESP_ERR_NOT_FOUND) {
                ESP_LOGW(TAG, "No servo responded");
            } else if (error != ESP_OK) {
                ESP_LOGE(
                    TAG,
                    "AT+READID failed: %s",
                    esp_err_to_name(error));
            } else {
                ESP_LOGI(TAG, "OK: ID scan completed");
            }
            send_run_over();
        } else if (strcmp(line.text, "AT+LAICLICKBUTTON?") == 0) {
            error = run_single_servo_button_action(
                AI_BUTTON_SERVO_ID,
                BUTTON_PRESS_HOLD_MS);

            if (error == ESP_OK) {
                const char response[] =
                    "OK\r\n+LAICLICKBUTTON:OK\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                fsr_log_peak("AI click button");
                ESP_LOGI(TAG, "AI button click completed");
            } else {
                const char response[] =
                    "+LAICLICKBUTTON:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                ESP_LOGE(
                    TAG,
                    "AI button click failed: %s",
                    esp_err_to_name(error));
            }
            send_run_over();
        } else if (strcmp(line.text, "AT+LAIDBLCLICKBUTTON?") == 0) {
            error = run_single_servo_button_action(
                AI_BUTTON_SERVO_ID,
                BUTTON_PRESS_HOLD_MS);
            if (error == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(BUTTON_DOUBLE_GAP_MS));
                error = run_single_servo_button_action(
                    AI_BUTTON_SERVO_ID,
                    BUTTON_PRESS_HOLD_MS);
            }

            if (error == ESP_OK) {
                const char response[] =
                    "OK\r\n+LAIDBLCLICKBUTTON:OK\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                ESP_LOGI(TAG, "AI button double-click completed");
            } else {
                const char response[] =
                    "+LAIDBLCLICKBUTTON:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                ESP_LOGE(
                    TAG,
                    "AI button double-click failed: %s",
                    esp_err_to_name(error));
            }
            send_run_over();
        } else if (strcmp(line.text, "AT+CLICKBUTTON?") == 0) {
            servo_id = 1;
            press_angle = servo_get_press_angle(1);
            fsr_monitor_reset_peak();
            error = s_action_handler(servo_id, press_angle);
            if (error == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(BUTTON_PRESS_HOLD_MS));
                error = s_action_handler(servo_id, 0);
                if (error == ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(
                        BUTTON_RETURN_SETTLE_MS));
                }
            }

            if (error == ESP_OK) {
                const char response[] =
                    "OK\r\n+LCLICKBUTTON:OK\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                fsr_log_peak("Click button");
                ESP_LOGI(
                    TAG,
                    "Button click completed: servo %d, %+.1f to 0 degrees",
                    servo_id,
                    press_angle);
            } else {
                const char response[] =
                    "+LCLICKBUTTON:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                ESP_LOGE(
                    TAG,
                    "Button click failed: %s",
                    esp_err_to_name(error));
            }
            send_run_over();
        } else if (strcmp(
                       line.text,
                       "AT+DBLCLICKBUTTON?") == 0) {
            servo_id = 1;
            press_angle = servo_get_press_angle(1);
            fsr_monitor_reset_peak();
            error = s_action_handler(servo_id, press_angle);
            if (error == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(BUTTON_PRESS_HOLD_MS));
                error = s_action_handler(servo_id, 0);
                if (error == ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(
                        BUTTON_RETURN_SETTLE_MS));
                }
            }
            if (error == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(BUTTON_DOUBLE_GAP_MS));
                error = s_action_handler(servo_id, press_angle);
            }
            if (error == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(BUTTON_PRESS_HOLD_MS));
                error = s_action_handler(servo_id, 0);
                if (error == ESP_OK) {
                    vTaskDelay(pdMS_TO_TICKS(
                        BUTTON_RETURN_SETTLE_MS));
                }
            }

            if (error == ESP_OK) {
                const char response[] =
                    "OK\r\n+LDBLCLICKBUTTON:OK\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                fsr_log_peak("Double-click button");
                ESP_LOGI(
                    TAG,
                    "Button double-click completed");
            } else {
                const char response[] =
                    "+LDBLCLICKBUTTON:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                ESP_LOGE(
                    TAG,
                    "Button double-click failed: %s",
                    esp_err_to_name(error));
            }
            send_run_over();
        } else if (parse_two_integer_parameters(
                       line.text,
                       "AT+LSETPRESSANGLE=",
                       &servo_id,
                       &angle)) {
            error = servo_set_press_angle(servo_id, angle);
            if (error == ESP_OK) {
                char response[64];
                snprintf(
                    response,
                    sizeof(response),
                    "OK\r\n+LSETPRESSANGLE:%d,%+.1f\r\n",
                    servo_id,
                    angle);
                uart_write_bytes(UART_NUM_0, response, strlen(response));
                ESP_LOGI(
                    TAG,
                    "Servo %d press angle saved: %+.1f degrees",
                    servo_id,
                    angle);
            } else {
                const char response[] = "+LSETPRESSANGLE:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                ESP_LOGE(
                    TAG,
                    "Set press angle failed: servo=%d angle=%+.1f, %s",
                    servo_id,
                    angle,
                    esp_err_to_name(error));
            }
            send_run_over();
        } else if (parse_id_command(
                       line.text,
                       "AT+LGETPRESSANGLE=",
                       &servo_id)) {
            float setting = 0.0f;
            error = servo_get_press_angle_setting(servo_id, &setting);
            if (error == ESP_OK) {
                char response[64];
                snprintf(
                    response,
                    sizeof(response),
                    "+LGETPRESSANGLE:%d,%+.1f\r\nOK\r\n",
                    servo_id,
                    setting);
                uart_write_bytes(UART_NUM_0, response, strlen(response));
            } else {
                const char response[] = "+LGETPRESSANGLE:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
            }
            send_run_over();
        } else if (parse_two_integer_parameters(
                       line.text,
                       "AT+ACTION=",
                       &servo_id,
                       &angle)) {
            fsr_monitor_reset_peak();
            error = s_action_handler(servo_id, angle);
            if (error == ESP_ERR_INVALID_ARG) {
                ESP_LOGW(
                    TAG,
                    "Invalid ID or angle: ID=%d, angle=%.1f",
                    servo_id,
                    angle);
            } else if (error != ESP_OK) {
                ESP_LOGE(
                    TAG,
                    "AT+ACTION failed for servo %d angle %.1f: %s",
                    servo_id,
                    angle,
                    esp_err_to_name(error));
            } else {
                vTaskDelay(pdMS_TO_TICKS(FSR_ACTION_CAPTURE_MS));
                fsr_log_peak("AT+ACTION");
                ESP_LOGI(
                    TAG,
                    "OK: servo %d moved to %.1f degrees",
                    servo_id,
                    angle);
            }
            send_run_over();
        } else {
            ESP_LOGW(TAG, "Invalid command: \"%s\"", line.text);
        }
    }
}

esp_err_t at_parser_start(
    QueueHandle_t line_queue,
    at_set_id_handler_t set_id_handler,
    at_action_handler_t action_handler,
    at_read_id_handler_t read_id_handler)
{
    if (line_queue == NULL ||
        set_id_handler == NULL ||
        action_handler == NULL ||
        read_id_handler == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_started) {
        return ESP_ERR_INVALID_STATE;
    }

    s_line_queue = line_queue;
    s_set_id_handler = set_id_handler;
    s_action_handler = action_handler;
    s_read_id_handler = read_id_handler;

    if (xTaskCreate(
            at_parse_task,
            "at_parse",
            AT_PARSER_TASK_STACK,
            NULL,
            AT_PARSER_TASK_PRIORITY,
            NULL) != pdPASS) {
        s_line_queue = NULL;
        s_set_id_handler = NULL;
        s_action_handler = NULL;
        s_read_id_handler = NULL;
        return ESP_ERR_NO_MEM;
    }

    s_started = true;
    return ESP_OK;
}
