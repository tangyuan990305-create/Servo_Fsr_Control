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
//2026.10.09 新增：压力闭环控制模块
#include "press_control.h"
#include "servo_task.h"
#include "system_commands.h"
#include "uart_receiver.h"

//2026.10.10 修改：标定窗口包含40点压力统计和多层函数调用，
//原3072字节栈不足，提升到8192字节避免at_parse栈溢出
#define AT_PARSER_TASK_STACK     8192
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
//2026.10.10 修改：平均压力允许向下5g测量误差，
//不再额外增加10g安全裕量。峰值只记录，不参与判定。
//2026.10.10 修改：根据-81.0~-80.6度各100次测试，
//-80.8度 max3 最小值约207.5g，-80.7度仅约28%窗口达到205g。
//因此舵机1改用205/280g作为稳定边界。
#define PRESS_MIN_1_G          155.0f
#define PRESS_MAX_1_G          260.0f
#define PRESS_PEAK_MIN_1_G     210.0f
#define PRESS_MIN_2_G          234.6f
#define PRESS_MAX_2_G          373.0f
#define PRESS_PEAK_MIN_2_G     0.0f
#define PRESS_MARGIN_G         0.0f
//2026.10.09 新增：压力闭环最长搜索时间，逐步逼近目标克重耗时较长
#define PRESS_CONTROL_TIMEOUT_MS 30000

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
static void fsr_log_measurement(
    const char *action,
    const press_result_t *result);
//2026.10.10 新增：run_fsr_repeat_test 在函数定义前调用该压力闭环动作，
//补充前向声明以通过编译
static esp_err_t run_single_servo_button_action(
    int servo_id,
    int hold_ms);
static esp_err_t run_single_servo_button_action_with_result(
    int servo_id,
    int hold_ms,
    press_result_t *result_out);

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
        count < 1 || count > 100) {
        return ESP_ERR_INVALID_ARG;
    }

    fsr_stats_reset();
    fsr_monitor_set_periodic_logging(false);

    esp_err_t error = ESP_OK;
    for (int index = 0; index < count; ++index) {
        ESP_LOGI(
            TAG,
            "FSR repeat test: %d/%d",
            index + 1,
            count);

        //2026.10.09 修改：原实现转到固定角度再回零，无法适配不同平台的
        //齿条安装位置。现改为压力闭环按压，达到目标克重后自动回零
        press_result_t result = {0};
        error = run_single_servo_button_action_with_result(
            servo_id,
            BUTTON_PRESS_HOLD_MS,
            &result);
        if (error != ESP_OK) {
            break;
        }

        fsr_log_measurement("Repeat test", &result);
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

//2026.10.10 新增：重复测试统计使用压力窗口max3，与标定判据保持一致
static void fsr_log_measurement(
    const char *action,
    const press_result_t *result)
{
    if (result == NULL) {
        return;
    }

    const float max3_g = result->reached_g;
    ++s_force_sample_count;
    s_force_sum_g += max3_g;
    s_force_last_g = max3_g;
    s_force_average_g =
        s_force_sum_g / (float)s_force_sample_count;

    if (s_force_sample_count == 1 || max3_g < s_force_min_g) {
        s_force_min_g = max3_g;
    }
    if (s_force_sample_count == 1 || max3_g > s_force_max_g) {
        s_force_max_g = max3_g;
    }

    ESP_LOGI(
        TAG,
        "%s FSR max3: %.0fg, peak=%.0fg, angle=%+.1f; "
        "samples=%u avg=%.0fg min=%.0fg max=%.0fg",
        action,
        max3_g,
        result->peak_g,
        result->final_angle_deg,
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

//2026.10.10 新增：解析 AT+FSRFIXED=<ID>,<角度>,<次数>
static bool parse_fixed_pressure_parameters(
    const char *line,
    const char *prefix,
    int *servo_id,
    float *angle_degrees,
    int *count)
{
    size_t prefix_length = strlen(prefix);
    const char *parameter = line + prefix_length;

    if (strncmp(line, prefix, prefix_length) != 0) {
        return false;
    }

    char *end = NULL;
    if (!parse_integer(parameter, &end, servo_id) || *end != ',') {
        return false;
    }

    errno = 0;
    char *angle_end = NULL;
    float angle = strtof(end + 1, &angle_end);
    if (errno == ERANGE || angle_end == end + 1 || *angle_end != ',') {
        return false;
    }

    //2026.10.13 新增：固定角度测试支持0.05度步进，
    //用于测试两个相邻舵机位置之间的机械临界点。
    float twentieth_steps = angle * 20.0f;
    int rounded_twentieth_steps = twentieth_steps >= 0.0f
        ? (int)(twentieth_steps + 0.5f)
        : (int)(twentieth_steps - 0.5f);
    float difference =
        twentieth_steps - (float)rounded_twentieth_steps;
    if (difference < 0.0f) {
        difference = -difference;
    }
    if (difference > 0.001f) {
        return false;
    }

    if (!parse_integer(angle_end + 1, &end, count) || *end != '\0') {
        return false;
    }

    *angle_degrees = angle;
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

//2026.10.10 新增：统一组装压力闭环参数
static esp_err_t build_press_request(
    int servo_id,
    int hold_ms,
    press_request_t *request)
{
    if (request == NULL || (servo_id != 1 && servo_id != 2)) {
        return ESP_ERR_INVALID_ARG;
    }

    request->servo_id = servo_id;
    request->min_g = servo_id == 1
        ? PRESS_MIN_1_G
        : PRESS_MIN_2_G;
    request->max_g = servo_id == 1
        ? PRESS_MAX_1_G
        : PRESS_MAX_2_G;
    request->peak_min_g = servo_id == 1
        ? PRESS_PEAK_MIN_1_G
        : PRESS_PEAK_MIN_2_G;
    request->margin_g = PRESS_MARGIN_G;
    request->max_angle_deg = servo_get_press_angle(servo_id);
    request->hold_ms = hold_ms;
    request->timeout_ms = PRESS_CONTROL_TIMEOUT_MS;
    return ESP_OK;
}

static esp_err_t run_single_servo_button_action_with_result(
    int servo_id,
    int hold_ms,
    press_result_t *result_out)
{
    //2026.10.10 修改：原实现转到固定角度后延时回零，无法确认是否达到目标压力。
    //现改为压力闭环，达到目标克重后由 press_control 自动回零。
    press_request_t request;
    esp_err_t error = build_press_request(
        servo_id,
        hold_ms,
        &request);
    if (error != ESP_OK) {
        return error;
    }
    press_result_t result = {0};

    error = press_control_execute(&request, &result);
    if (error == ESP_OK) {
        ESP_LOGI(
            TAG,
            "PRESS_RESULT: servo=%d target=%.1fg reached=%.1fg "
            "peak=%.1fg press_angle=%.1f elapsed=%dms",
            servo_id,
            result.target_g,
            result.reached_g,
            result.peak_g,
            result.final_angle_deg,
            result.elapsed_ms);
    } else {
        ESP_LOGE(
            TAG,
            "PRESS_RESULT_ERROR: servo=%d target=%.1fg "
            "reached=%.1fg peak=%.1fg press_angle=%.1f error=%s",
            servo_id,
            result.target_g,
            result.reached_g,
            result.peak_g,
            result.final_angle_deg,
            esp_err_to_name(error));
    }

    if (result_out != NULL) {
        *result_out = result;
    }
    return error;
}

static esp_err_t run_single_servo_button_action(
    int servo_id,
    int hold_ms)
{
    return run_single_servo_button_action_with_result(
        servo_id,
        hold_ms,
        NULL);
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
        //2026.10.20 新增：控制固定角度测试的逐点压力输出
        } else if (parse_integer_parameter(
                       line.text,
                       "AT+FSRTRACE=",
                       &value)) {
            if (value != 0 && value != 1) {
                const char response[] =
                    "+FSRTRACE:ERROR,invalid_parameter\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
            } else {
                press_control_set_trace_enabled(value == 1);
                if (value == 1) {
                    const char header[] =
                        "FSR_TRACE_HEADER,cycle,angle_deg,sample_index,"
                        "elapsed_ms,raw_mv,model_mv,force_g\r\n";
                    uart_write_bytes(
                        UART_NUM_0,
                        header,
                        sizeof(header) - 1);
                }
                const char *response =
                    value == 1
                        ? "+FSRTRACE:ON\r\n"
                        : "+FSRTRACE:OFF\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    strlen(response));
            }
            send_run_over();
        } else if (parse_integer_parameter(
                       line.text,
                       "AT+FSROFFSET=",
                       &value)) {
            //2026.10.21 新增：人工设置ADC偏移，避免零点采集覆盖标定值
            error = fsr_monitor_set_adc_offset_mv(value);
            if (error == ESP_OK) {
                char response[64];
                snprintf(
                    response,
                    sizeof(response),
                    "+FSROFFSET:%d\r\nOK\r\n",
                    value);
                uart_write_bytes(UART_NUM_0, response, strlen(response));
            } else {
                const char response[] =
                    "+FSROFFSET:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
            }
            send_run_over();
        } else if (parse_fixed_pressure_parameters(
                       line.text,
                       "AT+FSRFIXED=",
                       &servo_id,
                       &angle,
                       &value)) {
            int fixed_count = value;
            if ((servo_id != 1 && servo_id != 2) ||
                fixed_count < 1 || fixed_count > 100) {
                error = ESP_ERR_INVALID_ARG;
                const char response[] =
                    "+FSRFIXED:ERROR,invalid_parameter\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
                send_run_over();
            } else {
                error = ESP_OK;
                float min_avg = 0.0f, max_avg = 0.0f;
                float min_median = 0.0f, max_median = 0.0f;
                float min_peak = 0.0f, max_peak = 0.0f;
                float min_max3 = 0.0f, max_max3 = 0.0f;
                float min_final = 0.0f, max_final = 0.0f;
                double sum_avg = 0.0;
                double sum_median = 0.0;
                double sum_peak = 0.0;
                double sum_max3 = 0.0;
                double sum_final = 0.0;
                int completed = 0;

                fsr_monitor_set_periodic_logging(false);
                for (int index = 0;
                     index < fixed_count;
                     ++index) {
                    press_measurement_t measurement = {0};
                    ESP_LOGI(
                        TAG,
                        "FSR fixed test: %d/%d",
                        index + 1,
                        fixed_count);
                    //2026.10.20 新增：记录当前压力追踪所属测试周期
                    press_control_set_trace_cycle(index + 1);
                    error = press_control_measure_fixed(
                        servo_id,
                        angle,
                        &measurement);
                    if (error != ESP_OK) {
                        break;
                    }

                    if (completed == 0) {
                        min_avg = max_avg = measurement.average_g;
                        min_median = max_median = measurement.median_g;
                        min_peak = max_peak = measurement.peak_g;
                        min_max3 = max_max3 =
                            measurement.max_moving_average_g;
                        min_final = max_final =
                            measurement.final_average_g;
                    } else {
                        if (measurement.average_g < min_avg) {
                            min_avg = measurement.average_g;
                        }
                        if (measurement.average_g > max_avg) {
                            max_avg = measurement.average_g;
                        }
                        if (measurement.median_g < min_median) {
                            min_median = measurement.median_g;
                        }
                        if (measurement.median_g > max_median) {
                            max_median = measurement.median_g;
                        }
                        if (measurement.peak_g < min_peak) {
                            min_peak = measurement.peak_g;
                        }
                        if (measurement.peak_g > max_peak) {
                            max_peak = measurement.peak_g;
                        }
                        if (measurement.max_moving_average_g < min_max3) {
                            min_max3 =
                                measurement.max_moving_average_g;
                        }
                        if (measurement.max_moving_average_g > max_max3) {
                            max_max3 =
                                measurement.max_moving_average_g;
                        }
                        if (measurement.final_average_g < min_final) {
                            min_final = measurement.final_average_g;
                        }
                        if (measurement.final_average_g > max_final) {
                            max_final = measurement.final_average_g;
                        }
                    }

                    sum_avg += measurement.average_g;
                    sum_median += measurement.median_g;
                    sum_peak += measurement.peak_g;
                    sum_max3 += measurement.max_moving_average_g;
                    sum_final += measurement.final_average_g;
                    ++completed;

                    if (index + 1 < fixed_count) {
                        vTaskDelay(pdMS_TO_TICKS(
                            FSR_REPEAT_EXTRA_SETTLE_MS));
                    }
                }
                if (completed == fixed_count) {
                    char response[192];
                    snprintf(
                        response,
                        sizeof(response),
                        "+FSRFIXED:servo=%d,angle=%.2f,count=%d\r\n"
                        "+FSRFIXED:max3 avg=%.1f,min=%.1f,max=%.1f\r\n",
                        servo_id,
                        angle,
                        completed,
                        (float)(sum_max3 / completed),
                        min_max3,
                        max_max3);
                    uart_write_bytes(
                        UART_NUM_0,
                        response,
                        strlen(response));
                    snprintf(
                        response,
                        sizeof(response),
                        "+FSRFIXED:final avg=%.1f,min=%.1f,max=%.1f\r\n"
                        "+FSRFIXED:avg avg=%.1f,min=%.1f,max=%.1f\r\n",
                        (float)(sum_final / completed),
                        min_final,
                        max_final,
                        (float)(sum_avg / completed),
                        min_avg,
                        max_avg);
                    uart_write_bytes(
                        UART_NUM_0,
                        response,
                        strlen(response));
                    snprintf(
                        response,
                        sizeof(response),
                        "+FSRFIXED:median avg=%.1f,min=%.1f,max=%.1f\r\n"
                        "+FSRFIXED:peak avg=%.1f,min=%.1f,max=%.1f\r\n",
                        (float)(sum_median / completed),
                        min_median,
                        max_median,
                        (float)(sum_peak / completed),
                        min_peak,
                        max_peak);
                    uart_write_bytes(
                        UART_NUM_0,
                        response,
                        strlen(response));
                    const char response_ok[] = "OK\r\n";
                    uart_write_bytes(
                        UART_NUM_0,
                        response_ok,
                        sizeof(response_ok) - 1);
                } else {
                    char response[96];
                    snprintf(
                        response,
                        sizeof(response),
                        "+FSRFIXED:ERROR,completed=%d,error=%s\r\n",
                        completed,
                        esp_err_to_name(error));
                    uart_write_bytes(
                        UART_NUM_0,
                        response,
                        strlen(response));
                }
                //2026.10.20 修改：逐点追踪只对当前固定角度测试生效
                press_control_set_trace_enabled(false);
                send_run_over();
                //2026.10.10 修改：汇总和run over发送完后再恢复周期日志，
                //避免fsr_monitor插断FSRFIXED汇总
                (void)uart_wait_tx_done(
                    UART_NUM_0,
                    pdMS_TO_TICKS(100));
                fsr_monitor_set_periodic_logging(true);
            }
        } else if (parse_two_integer_parameters(
                       line.text,
                       "AT+FSRTEST=",
                       &servo_id,
                       &angle)) {
            int repeat_count = (int)angle;
            if ((float)repeat_count != angle ||
                repeat_count < 1 || repeat_count > 100) {
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
                    "+FSRTEST:servo=%d,count=%u,metric=max3,avg=%.0f,"
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
                "+FSR:%d,%+.0f,zero=%d,model=%d,offset=%.1f\r\n"
                "OK\r\n",
                fsr_monitor_get_current_mv(),
                fsr_monitor_get_current_grams(),
                fsr_monitor_get_zero_mv(),
                fsr_monitor_get_model_mv(),
                fsr_monitor_get_adc_offset_mv());
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
                //2026.10.10 修改：长按改为压力闭环，达到目标克重后自动回零
                error = run_single_servo_button_action(
                    1,
                    value + BUTTON_LONG_EXTRA_MS);
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
            //2026.10.10 修改：单击改为压力闭环，达到目标克重后自动回零
            error = run_single_servo_button_action(
                servo_id,
                BUTTON_PRESS_HOLD_MS);

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
            //2026.10.10 修改：双击的每一次按压均改为压力闭环
            error = run_single_servo_button_action(
                servo_id,
                BUTTON_PRESS_HOLD_MS);
            if (error == ESP_OK) {
                vTaskDelay(pdMS_TO_TICKS(BUTTON_DOUBLE_GAP_MS));
                error = run_single_servo_button_action(
                    servo_id,
                    BUTTON_PRESS_HOLD_MS);
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
                       "AT+LCALPRESSANGLE=",
                       &servo_id)) {
            press_request_t request;
            press_result_t result = {0};
            error = build_press_request(
                servo_id,
                BUTTON_PRESS_HOLD_MS,
                &request);
            if (error == ESP_OK) {
                error = press_control_calibrate(&request, &result);
            }

            if (error == ESP_OK) {
                char response[96];
                snprintf(
                    response,
                    sizeof(response),
                    "+LCALPRESSANGLE:%d,%.1f,%.1fg,%.1fg\r\n"
                    "OK\r\n",
                    servo_id,
                    result.final_angle_deg,
                    result.target_g,
                    result.reached_g);
                uart_write_bytes(UART_NUM_0, response, strlen(response));
            } else {
                char response[96];
                snprintf(
                    response,
                    sizeof(response),
                    "+LCALPRESSANGLE:ERROR,%s\r\n",
                    esp_err_to_name(error));
                uart_write_bytes(UART_NUM_0, response, strlen(response));
            }
            send_run_over();
        } else if (parse_id_command(
                       line.text,
                       "AT+LGETMINPRESSANGLE=",
                       &servo_id)) {
            float min_press_angle = 0.0f;
            error = servo_get_press_min_angle_setting(
                servo_id,
                &min_press_angle);
            if (error == ESP_OK) {
                char response[64];
                snprintf(
                    response,
                    sizeof(response),
                    "+LGETMINPRESSANGLE:%d,%+.1f\r\nOK\r\n",
                    servo_id,
                    min_press_angle);
                uart_write_bytes(UART_NUM_0, response, strlen(response));
            } else {
                const char response[] =
                    "+LGETMINPRESSANGLE:ERROR\r\n";
                uart_write_bytes(
                    UART_NUM_0,
                    response,
                    sizeof(response) - 1);
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
