//2026.10.09 新增：压力闭环控制所需的头文件
#include "press_control.h"

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fsr_monitor.h"
#include "servo_task.h"

//2026.10.10 修改：改为首次标定最小压力角度，后续直接复用并小范围修正
#define PRESS_CONTROL_TAG            "press_control"
#define PRESS_ADJUST_STEP_DEG        0.1f
//2026.10.10 修改：从开始移动起连续采集2000ms，
//与手动AT+ACTION的压力观察窗口保持一致
#define PRESS_CAPTURE_WINDOW_MS      2000
#define PRESS_CAPTURE_INTERVAL_MS    5
#define PRESS_CAPTURE_MAX_SAMPLES    400
#define PRESS_MOVING_AVG_POINTS      3
#define PRESS_FINAL_AVG_POINTS       5
//2026.10.10 修改：首次标定会从0°直接移动到最大安全角，
//不再使用只适合0.1°小步进的200ms超时，允许最长1500ms到位
#define PRESS_STEP_POSITION_TIMEOUT_MS 1500
#define PRESS_STEP_POSITION_POLL_MS  10
#define PRESS_CAL_STABLE_COUNT       3
#define PRESS_CAL_REQUIRED_PASS_COUNT 2
#define PRESS_USE_STABLE_COUNT       2
#define PRESS_CAL_MAX_STEPS          100
#define PRESS_CAL_COARSE_START_DEG   75.0f
#define PRESS_CAL_COARSE_STEP_DEG    1.0f
#define PRESS_CAL_MAX_VERIFY_STEPS   10
#define PRESS_USE_MAX_ADJUST_STEPS   40
#define PRESS_CAL_HOLD_MS            200
#define PRESS_RELEASE_POSITION_TIMEOUT_MS 1500
#define PRESS_RELEASE_POSITION_POLL_MS 20
#define PRESS_DEFAULT_MARGIN_G       10.0f
#define PRESS_DEFAULT_HOLD_MS        1500
#define PRESS_DEFAULT_TIMEOUT_MS     30000
#define PRESS_CAL_TIMEOUT_MS         60000
#define PRESS_ZERO_POSITION          2047.0f
#define PRESS_ZERO_POSITION_TOLERANCE 5
#define PRESS_POSITION_SPAN          2047.0f

//2026.10.10 新增：将目标角度换算为舵机位置寄存器值
static uint16_t press_control_angle_to_position(float angle_degrees)
{
    float scaled = angle_degrees * PRESS_POSITION_SPAN / 180.0f;
    int offset = scaled >= 0.0f
        ? (int)(scaled + 0.5f)
        : (int)(scaled - 0.5f);
    int position = (int)PRESS_ZERO_POSITION + offset;

    if (position < 0) {
        position = 0;
    } else if (position > 4095) {
        position = 4095;
    }
    return (uint16_t)position;
}

//2026.10.20 新增：固定角度测试的逐点压力追踪状态
static bool s_trace_enabled;
static int s_trace_cycle;
static int s_trace_pending_samples;
static float s_trace_angle_degrees;
static uint16_t s_trace_raw_mv[PRESS_CAPTURE_MAX_SAMPLES];
static int16_t s_trace_model_mv[PRESS_CAPTURE_MAX_SAMPLES];
static uint16_t s_trace_elapsed_ms[PRESS_CAPTURE_MAX_SAMPLES];
static float s_trace_force_g[PRESS_CAPTURE_MAX_SAMPLES];

void press_control_set_trace_enabled(bool enabled)
{
    s_trace_enabled = enabled;
    if (!enabled) {
        s_trace_cycle = 0;
    }
}

bool press_control_trace_is_enabled(void)
{
    return s_trace_enabled;
}

void press_control_set_trace_cycle(int cycle_index)
{
    s_trace_cycle = cycle_index;
}

//2026.10.20 新增：缓存原始压力，避免串口输出拖慢5ms采样
static void press_control_trace_sample(
    unsigned int elapsed_ms,
    int raw_mv,
    int model_mv,
    float grams)
{
    if (!s_trace_enabled) {
        return;
    }

    int index = s_trace_pending_samples;
    if (index < PRESS_CAPTURE_MAX_SAMPLES) {
        s_trace_raw_mv[index] = (uint16_t)raw_mv;
        s_trace_model_mv[index] = (int16_t)model_mv;
        s_trace_elapsed_ms[index] = (uint16_t)elapsed_ms;
        s_trace_force_g[index] = grams;
        ++s_trace_pending_samples;
    }
}

//2026.10.20 新增：舵机回零后统一输出逐点数据
static void press_control_trace_flush(void)
{
    if (!s_trace_enabled) {
        return;
    }

    for (int index = 0;
         index < s_trace_pending_samples;
         ++index) {
        char response[96];
        int length = snprintf(
            response,
            sizeof(response),
            "FSR_TRACE,%d,%.2f,%d,%u,%d,%d,%.1f\r\n",
            s_trace_cycle,
            s_trace_angle_degrees,
            index + 1,
            (unsigned int)s_trace_elapsed_ms[index],
            (int)s_trace_raw_mv[index],
            (int)s_trace_model_mv[index],
            s_trace_force_g[index]);
        if (length > 0) {
            if ((size_t)length >= sizeof(response)) {
                length = sizeof(response) - 1;
            }
            uart_write_bytes(UART_NUM_0, response, length);
        }
    }

    s_trace_pending_samples = 0;
}

//2026.10.10 新增：轮询舵机位置，确认运动完成
static esp_err_t press_control_wait_for_position(
    int servo_id,
    uint16_t target_position,
    int timeout_ms,
    int poll_interval_ms,
    const char *context,
    uint16_t *last_position,
    esp_err_t *last_read_error)
{
    const TickType_t start_tick = xTaskGetTickCount();
    uint16_t position = 0;
    esp_err_t read_error = ESP_ERR_TIMEOUT;

    while ((xTaskGetTickCount() - start_tick) <
           pdMS_TO_TICKS(timeout_ms)) {
        read_error = servo_read_position(servo_id, &position);
        if (read_error == ESP_OK) {
            int target_offset =
                (int)position - (int)target_position;
            if (target_offset < 0) {
                target_offset = -target_offset;
            }
            if (target_offset <= PRESS_ZERO_POSITION_TOLERANCE) {
                if (last_position != NULL) {
                    *last_position = position;
                }
                if (last_read_error != NULL) {
                    *last_read_error = ESP_OK;
                }
                return ESP_OK;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(poll_interval_ms));
    }

    if (last_position != NULL) {
        *last_position = position;
    }
    if (last_read_error != NULL) {
        *last_read_error = read_error;
    }

    int target_offset = -1;
    if (read_error == ESP_OK) {
        target_offset = (int)position - (int)target_position;
        if (target_offset < 0) {
            target_offset = -target_offset;
        }
    }

    ESP_LOGW(
        PRESS_CONTROL_TAG,
        "PRESS_POSITION_TIMEOUT: servo=%d context=%s target=%u "
        "position=%u offset=%d read_error=%s",
        servo_id,
        context,
        target_position,
        position,
        target_offset,
        esp_err_to_name(read_error));

    return read_error == ESP_OK ? ESP_ERR_TIMEOUT : read_error;
}

//2026.10.09 新增：返回舵机到释放位置，避免失败后继续保持按压
static esp_err_t press_control_release(
    int servo_id,
    const char *reason)
{
    ESP_LOGI(
        PRESS_CONTROL_TAG,
        "PRESS_RELEASE_BEGIN: servo=%d reason=%s command=0.0deg",
        servo_id,
        reason);

    esp_err_t error = servo_set_angle(servo_id, 0.0f);
    if (error != ESP_OK) {
        ESP_LOGE(
            PRESS_CONTROL_TAG,
            "PRESS_RELEASE_ERROR: servo=%d reason=%s error=%s",
            servo_id,
            reason,
            esp_err_to_name(error));
        return error;
    }

    uint16_t position = 0;
    esp_err_t read_error = ESP_OK;
    esp_err_t wait_error = press_control_wait_for_position(
        servo_id,
        (uint16_t)PRESS_ZERO_POSITION,
        PRESS_RELEASE_POSITION_TIMEOUT_MS,
        PRESS_RELEASE_POSITION_POLL_MS,
        reason,
        &position,
        &read_error);

    if (wait_error == ESP_OK && read_error == ESP_OK) {
        int position_offset = (int)position - (int)PRESS_ZERO_POSITION;
        if (position_offset < 0) {
            position_offset = -position_offset;
        }
        ESP_LOGI(
            PRESS_CONTROL_TAG,
            "PRESS_RELEASE_DONE: servo=%d reason=%s position=%u "
            "expected=%.0f offset=%d confirmed=YES",
            servo_id,
            reason,
            position,
            PRESS_ZERO_POSITION,
            position_offset);
        return ESP_OK;
    }

    if (read_error == ESP_OK) {
        int position_offset = (int)position - (int)PRESS_ZERO_POSITION;
        if (position_offset < 0) {
            position_offset = -position_offset;
        }
        ESP_LOGW(
            PRESS_CONTROL_TAG,
            "PRESS_RELEASE_DONE: servo=%d reason=%s position=%u "
            "expected=%.0f offset=%d confirmed=NO",
            servo_id,
            reason,
            position,
            PRESS_ZERO_POSITION,
            position_offset);
    } else {
        ESP_LOGW(
            PRESS_CONTROL_TAG,
            "PRESS_RELEASE_DONE: servo=%d reason=%s "
            "position_read_error=%s",
            servo_id,
            reason,
            esp_err_to_name(read_error));
    }

    return wait_error;
}

//2026.10.10 新增：统一校验压力控制参数
static esp_err_t press_control_validate_request(
    const press_request_t *request)
{
    if (request == NULL ||
        (request->servo_id != 1 && request->servo_id != 2) ||
        request->min_g <= 0.0f ||
        request->max_g < request->min_g ||
        request->peak_min_g < 0.0f ||
        request->margin_g < 0.0f ||
        request->max_angle_deg == 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    return ESP_OK;
}

//2026.10.10 新增：初始化结果对象
static void press_control_init_result(
    const press_request_t *request,
    press_result_t *result)
{
    result->target_g = request->min_g + request->margin_g;
    result->reached_g = 0.0f;
    result->peak_g = 0.0f;
    result->final_angle_deg = 0.0f;
    result->elapsed_ms = 0;
    result->target_reached = false;
}

//2026.10.10 新增：从开始移动到移动完成后的2000ms压力窗口
static esp_err_t press_control_move_and_capture(
    int servo_id,
    float angle_degrees,
    press_measurement_t *measurement)
{
    if (measurement == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    fsr_monitor_reset_peak();
    esp_err_t error = servo_set_angle(servo_id, angle_degrees);
    if (error != ESP_OK) {
        return error;
    }

    float samples[PRESS_CAPTURE_MAX_SAMPLES];
    int sample_count = 0;
    const TickType_t start_tick = xTaskGetTickCount();
    if (s_trace_enabled) {
        s_trace_pending_samples = 0;
        s_trace_angle_degrees = angle_degrees;
    }

    while (sample_count < PRESS_CAPTURE_MAX_SAMPLES &&
           (xTaskGetTickCount() - start_tick) <
               pdMS_TO_TICKS(PRESS_CAPTURE_WINDOW_MS)) {
        TickType_t sample_tick = xTaskGetTickCount();
        unsigned int elapsed_ms = (unsigned int)(
            (sample_tick - start_tick) * portTICK_PERIOD_MS);
        int raw_mv = 0;
        int model_mv = 0;
        float grams = 0.0f;
        fsr_monitor_get_current_snapshot(
            &raw_mv,
            &model_mv,
            &grams);

        samples[sample_count] = grams;
        ++sample_count;
        //2026.10.20 新增：输出当前采样点的原始ADC和校正结果
        press_control_trace_sample(
            elapsed_ms,
            raw_mv,
            model_mv,
            grams);

        //2026.10.20 修改：采样阶段只缓存，串口数据在回零后统一输出
        vTaskDelay(pdMS_TO_TICKS(PRESS_CAPTURE_INTERVAL_MS));
    }

    if (sample_count == 0) {
        return ESP_ERR_INVALID_STATE;
    }

    float sum_g = 0.0f;
    float peak_g = 0.0f;
    for (int i = 0; i < sample_count; ++i) {
        sum_g += samples[i];
        if (samples[i] > peak_g) {
            peak_g = samples[i];
        }
    }
    float monitor_peak_g = fsr_monitor_get_peak_grams();
    if (monitor_peak_g > peak_g) {
        peak_g = monitor_peak_g;
    }

    int moving_count = PRESS_MOVING_AVG_POINTS < sample_count
        ? PRESS_MOVING_AVG_POINTS
        : sample_count;
    float max_moving_average_g = 0.0f;
    for (int start = 0;
         start + moving_count <= sample_count;
         ++start) {
        float moving_sum = 0.0f;
        for (int i = 0; i < moving_count; ++i) {
            moving_sum += samples[start + i];
        }
        float moving_average = moving_sum / (float)moving_count;
        if (moving_average > max_moving_average_g) {
            max_moving_average_g = moving_average;
        }
    }

    int final_count = PRESS_FINAL_AVG_POINTS < sample_count
        ? PRESS_FINAL_AVG_POINTS
        : sample_count;
    float final_sum = 0.0f;
    for (int i = sample_count - final_count;
         i < sample_count;
         ++i) {
        final_sum += samples[i];
    }

    float sorted[PRESS_CAPTURE_MAX_SAMPLES];
    for (int i = 0; i < sample_count; ++i) {
        sorted[i] = samples[i];
    }
    for (int i = 1; i < sample_count; ++i) {
        float value = sorted[i];
        int position = i - 1;
        while (position >= 0 && sorted[position] > value) {
            sorted[position + 1] = sorted[position];
            --position;
        }
        sorted[position + 1] = value;
    }

    measurement->sample_count = sample_count;
    measurement->average_g = sum_g / (float)sample_count;
    measurement->median_g = sample_count % 2 == 0
        ? (sorted[sample_count / 2 - 1] +
           sorted[sample_count / 2]) / 2.0f
        : sorted[sample_count / 2];
    measurement->peak_g = peak_g;
    measurement->max_moving_average_g = max_moving_average_g;
    measurement->final_average_g =
        final_sum / (float)final_count;

    ESP_LOGI(
        PRESS_CONTROL_TAG,
        //2026.10.13 修改：固定角度测试支持0.05度，日志保留两位角度小数
        "PRESS_CAPTURE: angle=%.2f samples=%d avg=%.1fg "
        "median=%.1fg peak=%.1fg max3=%.1fg final=%.1fg",
        angle_degrees,
        measurement->sample_count,
        measurement->average_g,
        measurement->median_g,
        measurement->peak_g,
        measurement->max_moving_average_g,
        measurement->final_average_g);

    uint16_t position = 0;
    esp_err_t read_error = ESP_OK;
    error = press_control_wait_for_position(
        servo_id,
        press_control_angle_to_position(angle_degrees),
        PRESS_STEP_POSITION_TIMEOUT_MS,
        PRESS_STEP_POSITION_POLL_MS,
        "capture_position",
        &position,
        &read_error);
    return error;
}

//2026.10.10 新增：压力失败原因分类
static const char *press_control_failure_reason(
    const press_measurement_t *measurement,
    float lower_limit_g,
    float upper_limit_g,
    esp_err_t error)
{
    if (error != ESP_OK &&
        error != ESP_ERR_TIMEOUT &&
        error != ESP_ERR_NOT_FOUND) {
        return "CONTROL_ERROR";
    }
    if (measurement->max_moving_average_g < lower_limit_g) {
        return "PRESSURE_UNDER_RANGE";
    }
    if (measurement->final_average_g > upper_limit_g) {
        return "PRESSURE_OVER_RANGE";
    }
    return "TARGET_WINDOW_NOT_STABLE";
}

//2026.10.10 新增：判断一次2000ms压力窗口是否满足触发和上限条件
static bool press_control_measurement_passes(
    const press_measurement_t *measurement,
    float target_g,
    float peak_min_g,
    float upper_limit_g)
{
    return measurement->max_moving_average_g >= target_g &&
        measurement->peak_g >= peak_min_g &&
        measurement->final_average_g <= upper_limit_g;
}

//2026.10.10 新增：执行多次2000ms窗口，按通过次数进行投票判定
static esp_err_t press_control_verify_majority(
    int servo_id,
    float angle_degrees,
    const char *context,
    float target_g,
    float peak_min_g,
    float upper_limit_g,
    int attempts,
    int required_passes,
    press_measurement_t *best_measurement,
    press_measurement_t *last_measurement,
    int *pass_count_out)
{
    int pass_count = 0;
    press_measurement_t best = {0};
    press_measurement_t last = {0};

    for (int attempt = 0; attempt < attempts; ++attempt) {
        press_measurement_t measurement = {0};
        //2026.10.10 修改：验证窗口改为完整点击循环，
        //每次先回到0°，避免上一个角度的机械残余影响判定
        esp_err_t error = press_control_measure_fixed(
            servo_id,
            angle_degrees,
            &measurement);
        if (error != ESP_OK) {
            return error;
        }
        last = measurement;

        bool passed = press_control_measurement_passes(
            &measurement,
            target_g,
            peak_min_g,
            upper_limit_g);
        if (passed) {
            ++pass_count;
            best = measurement;
        }

        ESP_LOGI(
            PRESS_CONTROL_TAG,
            "PRESS_CAL_%s: servo=%d angle=%.2f attempt=%d/%d "
            "max3=%.1fg final=%.1fg result=%s",
            context,
            servo_id,
            angle_degrees,
            attempt + 1,
            attempts,
            measurement.max_moving_average_g,
            measurement.final_average_g,
            passed ? "PASS" : "FAIL");
    }

    if (best_measurement != NULL) {
        *best_measurement = best;
    }
    if (last_measurement != NULL) {
        *last_measurement = last;
    }
    if (pass_count_out != NULL) {
        *pass_count_out = pass_count;
    }

    ESP_LOGI(
        PRESS_CONTROL_TAG,
        "PRESS_CAL_VOTE: context=%s angle=%.2f passed=%d/%d "
        "required=%d",
        context,
        angle_degrees,
        pass_count,
        attempts,
        required_passes);

    return ESP_OK;
}

//2026.10.10 修改：从安全角度开始向按键方向搜索，
//粗调每步1°，检测到压力后使用0.1°精调。每次测量均完整回零。
esp_err_t press_control_calibrate(
    const press_request_t *request,
    press_result_t *result)
{
    esp_err_t error = press_control_validate_request(request);
    if (error != ESP_OK || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    fsr_monitor_set_periodic_logging(false);

    const int servo_id = request->servo_id;
    const float direction =
        request->max_angle_deg < 0.0f ? -1.0f : 1.0f;
    const float max_abs_angle = fabsf(request->max_angle_deg);
    const float lower_limit_g = request->min_g;
    const float target_g = request->min_g + request->margin_g;
    const float upper_limit_g = request->max_g;
    const TickType_t start_tick = xTaskGetTickCount();

    press_control_init_result(request, result);
    ESP_LOGI(
        PRESS_CONTROL_TAG,
        "PRESS_CAL_BEGIN: servo=%d hard_max=%.1f "
        "window=%.1f..%.1fg target=%.1fg peak_min=%.1fg",
        servo_id,
        request->max_angle_deg,
        lower_limit_g,
        upper_limit_g,
        target_g,
        request->peak_min_g);

    error = press_control_release(servo_id, "calibration_initial");
    if (error != ESP_OK) {
        fsr_monitor_set_periodic_logging(true);
        return error;
    }

    press_measurement_t last_measurement = {0};
    float current_angle_deg =
        direction * (PRESS_CAL_COARSE_START_DEG < max_abs_angle
            ? PRESS_CAL_COARSE_START_DEG
            : max_abs_angle);
    bool pressure_detected = false;

    //粗调：从安全角度每次向按压方向增加1°，直到检测到压力
    for (int step = 0;
         step <= PRESS_CAL_MAX_STEPS &&
         fabsf(current_angle_deg) <= max_abs_angle;
         ++step) {
        if ((xTaskGetTickCount() - start_tick) >=
            pdMS_TO_TICKS(PRESS_CAL_TIMEOUT_MS)) {
            error = ESP_ERR_TIMEOUT;
            break;
        }

        press_measurement_t measurement = {0};
        error = press_control_measure_fixed(
            servo_id,
            current_angle_deg,
            &measurement);
        if (error != ESP_OK) {
            break;
        }
        last_measurement = measurement;
        result->reached_g = measurement.max_moving_average_g;

        bool passed = press_control_measurement_passes(
            &measurement,
            target_g,
            request->peak_min_g,
            upper_limit_g);
        ESP_LOGI(
            PRESS_CONTROL_TAG,
            "PRESS_CAL_COARSE: servo=%d angle=%.2f "
            "max3=%.1fg peak=%.1fg final=%.1fg result=%s",
            servo_id,
            current_angle_deg,
            measurement.max_moving_average_g,
            measurement.peak_g,
            measurement.final_average_g,
            passed ? "PASS" : "FAIL");

        if (passed) {
            pressure_detected = true;
            break;
        }

        if (measurement.max_moving_average_g < target_g &&
            measurement.peak_g < request->peak_min_g &&
            measurement.final_average_g <= upper_limit_g) {
            current_angle_deg +=
                direction * PRESS_CAL_COARSE_STEP_DEG;
            continue;
        }

        //已经进入压力区或超过上限，交给0.1°精调
        pressure_detected = true;
        break;
    }

    bool verified = false;
    float verify_angle_deg = current_angle_deg;
    press_measurement_t last_pass_measurement = {0};

    if (pressure_detected) {
        bool has_last_pass = false;
        float fine_angle_deg = current_angle_deg;

        //精调：向压力减小方向回退0.1°，记录最后通过角度
        for (int step = 0;
             step <= PRESS_CAL_MAX_STEPS &&
             fine_angle_deg * direction > 0.0f;
             ++step) {
            if ((xTaskGetTickCount() - start_tick) >=
                pdMS_TO_TICKS(PRESS_CAL_TIMEOUT_MS)) {
                error = ESP_ERR_TIMEOUT;
                break;
            }

            press_measurement_t measurement = {0};
            error = press_control_measure_fixed(
                servo_id,
                fine_angle_deg,
                &measurement);
            if (error != ESP_OK) {
                break;
            }
            last_measurement = measurement;
            result->reached_g = measurement.max_moving_average_g;

            bool passed = press_control_measurement_passes(
                &measurement,
                target_g,
                request->peak_min_g,
                upper_limit_g);
            ESP_LOGI(
            PRESS_CONTROL_TAG,
            "PRESS_CAL_FINE: servo=%d angle=%.2f "
            "max3=%.1fg peak=%.1fg final=%.1fg result=%s",
            servo_id,
            fine_angle_deg,
            measurement.max_moving_average_g,
            measurement.peak_g,
            measurement.final_average_g,
            passed ? "PASS" : "FAIL");

            if (passed) {
                has_last_pass = true;
                verify_angle_deg = fine_angle_deg;
                last_pass_measurement = measurement;
                fine_angle_deg -=
                    direction * PRESS_ADJUST_STEP_DEG;
                continue;
            }

            if (measurement.max_moving_average_g < target_g ||
                measurement.peak_g < request->peak_min_g) {
                fine_angle_deg +=
                    direction * PRESS_ADJUST_STEP_DEG;
                if (fabsf(fine_angle_deg) > max_abs_angle) {
                    break;
                }
                continue;
            }

            //压力仍超过上限，继续向压力减小方向回退
            fine_angle_deg -= direction * PRESS_ADJUST_STEP_DEG;
        }

        if (has_last_pass) {
            float candidate_angle_deg = verify_angle_deg;
            for (int attempt = 0;
                 attempt <= PRESS_CAL_MAX_VERIFY_STEPS;
                 ++attempt) {
                if (fabsf(candidate_angle_deg) > max_abs_angle) {
                    candidate_angle_deg = request->max_angle_deg;
                }

                int verify_pass_count = 0;
                press_measurement_t verify_best = {0};
                error = press_control_verify_majority(
                    servo_id,
                    candidate_angle_deg,
                    attempt == 0 ? "VERIFY" : "SAFETY",
                    target_g,
                    request->peak_min_g,
                    upper_limit_g,
                    PRESS_CAL_STABLE_COUNT,
                    PRESS_CAL_REQUIRED_PASS_COUNT,
                    &verify_best,
                    &last_measurement,
                    &verify_pass_count);
                if (error != ESP_OK) {
                    break;
                }
                if (verify_pass_count >=
                    PRESS_CAL_REQUIRED_PASS_COUNT) {
                    verify_angle_deg = candidate_angle_deg;
                    last_pass_measurement = verify_best;
                    verified = true;
                    break;
                }

                if (fabsf(candidate_angle_deg -
                          request->max_angle_deg) < 0.05f) {
                    break;
                }
                candidate_angle_deg +=
                    direction * PRESS_ADJUST_STEP_DEG;
            }
        } else {
            error = ESP_ERR_NOT_FOUND;
        }
    } else if (error == ESP_OK) {
        error = ESP_ERR_NOT_FOUND;
    }

    if (verified) {
        error = servo_set_press_min_angle(
            servo_id,
            verify_angle_deg);
        if (error != ESP_OK) {
            fsr_monitor_set_periodic_logging(true);
            return error;
        }

        result->target_reached = true;
        result->reached_g =
            last_pass_measurement.max_moving_average_g;
        result->final_angle_deg = verify_angle_deg;
        result->peak_g = last_pass_measurement.peak_g;
        result->elapsed_ms = (int)(
            (xTaskGetTickCount() - start_tick) *
            portTICK_PERIOD_MS);
        ESP_LOGI(
            PRESS_CONTROL_TAG,
            "PRESS_CAL_SAVED: servo=%d angle=%.2f "
            "target=%.1fg max3=%.1fg final=%.1fg peak=%.1fg",
            servo_id,
            verify_angle_deg,
            target_g,
            last_pass_measurement.max_moving_average_g,
            last_pass_measurement.final_average_g,
            result->peak_g);

        error = press_control_release(
            servo_id,
            "calibration_saved");
        fsr_monitor_set_periodic_logging(true);
        return error;
    }

    result->peak_g = last_measurement.peak_g;
    result->final_angle_deg = verify_angle_deg;
    result->elapsed_ms = (int)(
        (xTaskGetTickCount() - start_tick) *
        portTICK_PERIOD_MS);
    const char *reason = press_control_failure_reason(
        &last_measurement,
        lower_limit_g,
        upper_limit_g,
        error);
    ESP_LOGE(
        PRESS_CONTROL_TAG,
        "PRESS_CAL_FAILED: servo=%d reason=%s target=%.1fg "
        "window=%.1f..%.1fg max3=%.1fg final=%.1fg "
        "peak=%.1fg press_angle=%.2f error=%s",
        servo_id,
        reason,
        target_g,
        lower_limit_g,
        upper_limit_g,
        last_measurement.max_moving_average_g,
        last_measurement.final_average_g,
        last_measurement.peak_g,
        result->final_angle_deg,
        esp_err_to_name(error));
    (void)press_control_release(servo_id, "calibration_failed");
    fsr_monitor_set_periodic_logging(true);
    return error == ESP_OK ? ESP_FAIL : error;
}

//2026.10.10 新增：使用已保存角度执行点击，必要时进行0.1度小范围修正
esp_err_t press_control_execute(
    const press_request_t *request,
    press_result_t *result)
{
    esp_err_t error = press_control_validate_request(request);
    if (error != ESP_OK || result == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    const int servo_id = request->servo_id;
    const float direction =
        request->max_angle_deg < 0.0f ? -1.0f : 1.0f;
    const float max_abs_angle = fabsf(request->max_angle_deg);
    const float lower_limit_g = request->min_g;
    const float target_g = request->min_g + request->margin_g;
    const float upper_limit_g = request->max_g;
    const int hold_ms = request->hold_ms > 0
        ? request->hold_ms
        : PRESS_DEFAULT_HOLD_MS;
    TickType_t start_tick = xTaskGetTickCount();

    press_control_init_result(request, result);

    float current_angle_deg = 0.0f;
    if (!servo_has_press_min_angle(servo_id) ||
        servo_get_press_min_angle_setting(
            servo_id,
            &current_angle_deg) != ESP_OK ||
        fabsf(current_angle_deg) > max_abs_angle + 0.01f) {
        ESP_LOGW(
            PRESS_CONTROL_TAG,
            "PRESS_CAL_REQUIRED: servo=%d no valid saved angle",
            servo_id);
        error = press_control_calibrate(request, result);
        if (error != ESP_OK) {
            return error;
        }
        press_control_init_result(request, result);
        start_tick = xTaskGetTickCount();
    }

    error = press_control_release(servo_id, "initial");
    if (error != ESP_OK) {
        return error;
    }
    fsr_monitor_reset_peak();

    error = servo_get_press_min_angle_setting(
        servo_id,
        &current_angle_deg);
    if (error != ESP_OK) {
        return error;
    }
    float saved_angle_deg = current_angle_deg;

    int stable_sample_count = 0;
    press_measurement_t last_measurement = {0};
    for (int step = 0; step <= PRESS_USE_MAX_ADJUST_STEPS; ++step) {
        if ((xTaskGetTickCount() - start_tick) >=
            pdMS_TO_TICKS(request->timeout_ms > 0
                ? request->timeout_ms
                : PRESS_DEFAULT_TIMEOUT_MS)) {
            error = ESP_ERR_TIMEOUT;
            break;
        }

        press_measurement_t measurement = {0};
        error = press_control_move_and_capture(
            servo_id,
            current_angle_deg,
            &measurement);
        if (error != ESP_OK) {
            break;
        }
        last_measurement = measurement;
        result->reached_g = measurement.max_moving_average_g;

        if (measurement.max_moving_average_g >= target_g &&
            measurement.peak_g >= request->peak_min_g &&
            measurement.final_average_g <= upper_limit_g) {
            ++stable_sample_count;
            if (stable_sample_count >= PRESS_USE_STABLE_COUNT) {
                //2026.10.10 新增：只有角度发生变化时才写入NVS，避免重复测试磨损Flash
                if (fabsf(current_angle_deg - saved_angle_deg) >= 0.05f) {
                    error = servo_set_press_min_angle(
                        servo_id,
                        current_angle_deg);
                    if (error != ESP_OK) {
                        break;
                    }
                    saved_angle_deg = current_angle_deg;
                }

                result->target_reached = true;
                result->final_angle_deg = current_angle_deg;
                vTaskDelay(pdMS_TO_TICKS(hold_ms));
                result->peak_g = measurement.peak_g;
                result->elapsed_ms = (int)(
                    (xTaskGetTickCount() - start_tick) *
                    portTICK_PERIOD_MS);
                ESP_LOGI(
                    PRESS_CONTROL_TAG,
                    "PRESS_TARGET_REACHED: servo=%d target=%.1fg "
                    "max3=%.1fg final=%.1fg peak=%.1fg "
                    "press_angle=%.2f",
                    servo_id,
                    target_g,
                    measurement.max_moving_average_g,
                    measurement.final_average_g,
                    measurement.peak_g,
                    current_angle_deg);
                error = press_control_release(
                    servo_id,
                    "target_reached");
                return error;
            }

            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        stable_sample_count = 0;
        if (measurement.max_moving_average_g < target_g ||
            measurement.peak_g < request->peak_min_g) {
            current_angle_deg += direction * PRESS_ADJUST_STEP_DEG;
        } else {
            current_angle_deg -= direction * PRESS_ADJUST_STEP_DEG;
        }

        ESP_LOGI(
            PRESS_CONTROL_TAG,
            "PRESS_ADJUST: servo=%d max3=%.1fg peak=%.1fg "
            "final=%.1fg "
            "next_angle=%.2f",
            servo_id,
            measurement.max_moving_average_g,
            measurement.peak_g,
            measurement.final_average_g,
            current_angle_deg);

        if (fabsf(current_angle_deg) > max_abs_angle) {
            error = ESP_ERR_TIMEOUT;
            break;
        }
    }

    result->peak_g = fsr_monitor_get_peak_grams();
    result->final_angle_deg = current_angle_deg;
    result->elapsed_ms = (int)(
        (xTaskGetTickCount() - start_tick) *
        portTICK_PERIOD_MS);
    const char *reason = press_control_failure_reason(
        &last_measurement,
        lower_limit_g,
        upper_limit_g,
        error);
    ESP_LOGE(
        PRESS_CONTROL_TAG,
        "PRESS_FAILED: servo=%d reason=%s target=%.1fg "
        "window=%.1f..%.1fg max3=%.1fg final=%.1fg "
        "peak=%.1fg press_angle=%.2f error=%s",
        servo_id,
        reason,
        target_g,
        lower_limit_g,
        upper_limit_g,
        last_measurement.max_moving_average_g,
        last_measurement.final_average_g,
        last_measurement.peak_g,
        result->final_angle_deg,
        esp_err_to_name(error));
    (void)press_control_release(servo_id, "abort");
    return error == ESP_OK ? ESP_FAIL : error;
}

//2026.10.10 新增：固定角度采集一次2000ms压力窗口，
//用于AT+FSRFIXED压力分布测试，不执行标定或角度自动修正
esp_err_t press_control_measure_fixed(
    int servo_id,
    float angle_degrees,
    press_measurement_t *measurement)
{
    if ((servo_id != 1 && servo_id != 2) ||
        measurement == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    float max_angle_degrees = servo_get_press_angle(servo_id);
    if (max_angle_degrees == 0.0f ||
        angle_degrees * max_angle_degrees <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    float max_abs_angle = fabsf(max_angle_degrees);
    if (fabsf(angle_degrees) > max_abs_angle + 0.01f) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = press_control_release(
        servo_id,
        "fixed_initial");
    if (error != ESP_OK) {
        return error;
    }

    error = press_control_move_and_capture(
        servo_id,
        angle_degrees,
        measurement);
    esp_err_t release_error = press_control_release(
        servo_id,
        "fixed_done");
    //2026.10.20 新增：回零后输出本次完整逐点曲线
    press_control_trace_flush();

    return error != ESP_OK ? error : release_error;
}
