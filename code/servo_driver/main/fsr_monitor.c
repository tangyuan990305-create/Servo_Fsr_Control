#include "fsr_monitor.h"

#include <stddef.h>
#include <stdint.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fsr_sensor.h"

#define FSR_MONITOR_TASK_STACK      3072
#define FSR_MONITOR_TASK_PRIORITY   4
#define FSR_MONITOR_PERIOD_MS       2
#define FSR_MONITOR_LOG_PERIOD_MS   500
#define FSR_MONITOR_FILTER_SIZE     5

static const char *TAG = "fsr_monitor";

static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_task;
static int s_current_mv;
static int s_current_filtered_mv;
static float s_current_grams;
static int s_peak_mv;
static float s_peak_grams;
static TickType_t s_peak_reset_tick;
static TickType_t s_peak_tick;
static bool s_periodic_logging = true;
static uint16_t s_voltage_history[FSR_MONITOR_FILTER_SIZE];
static size_t s_voltage_history_index;
static bool s_voltage_history_initialized;

//2026.10.21 新增：对最近5次ADC电压取中值，去除单点尖峰
static int fsr_monitor_median_voltage(int voltage_mv)
{
    if (!s_voltage_history_initialized) {
        for (size_t index = 0;
             index < FSR_MONITOR_FILTER_SIZE;
             ++index) {
            s_voltage_history[index] = (uint16_t)voltage_mv;
        }
        s_voltage_history_initialized = true;
    }

    s_voltage_history[s_voltage_history_index] =
        (uint16_t)voltage_mv;
    s_voltage_history_index =
        (s_voltage_history_index + 1) % FSR_MONITOR_FILTER_SIZE;

    int sorted[FSR_MONITOR_FILTER_SIZE];
    for (size_t index = 0;
         index < FSR_MONITOR_FILTER_SIZE;
         ++index) {
        sorted[index] = (int)s_voltage_history[index];
    }
    for (size_t index = 1;
         index < FSR_MONITOR_FILTER_SIZE;
         ++index) {
        int value = sorted[index];
        size_t position = index;
        while (position > 0 && sorted[position - 1] > value) {
            sorted[position] = sorted[position - 1];
            --position;
        }
        sorted[position] = value;
    }

    return sorted[FSR_MONITOR_FILTER_SIZE / 2];
}

static void fsr_monitor_delay(void)
{
    TickType_t ticks = pdMS_TO_TICKS(FSR_MONITOR_PERIOD_MS);
    if (ticks == 0) {
        ticks = 1;
    }
    vTaskDelay(ticks);
}

static void fsr_monitor_task(void *argument)
{
    (void)argument;
    TickType_t last_log_tick = xTaskGetTickCount();

    while (true) {
        int voltage_mv = 0;
        esp_err_t error = fsr_sensor_read_mv(&voltage_mv);
        if (error != ESP_OK) {
            ESP_LOGW(
                TAG,
                "ADC read failed: %s",
                esp_err_to_name(error));
            fsr_monitor_delay();
            continue;
        }

        int filtered_mv = fsr_monitor_median_voltage(voltage_mv);
        float grams = fsr_sensor_mv_to_grams(filtered_mv);
        TickType_t now = xTaskGetTickCount();

        taskENTER_CRITICAL(&s_state_lock);
        s_current_mv = voltage_mv;
        s_current_filtered_mv = filtered_mv;
        s_current_grams = grams;
        if (grams > s_peak_grams) {
            s_peak_mv = voltage_mv;
            s_peak_grams = grams;
            s_peak_tick = now;
        }
        taskEXIT_CRITICAL(&s_state_lock);

        if (s_periodic_logging &&
            (now - last_log_tick) >=
            pdMS_TO_TICKS(FSR_MONITOR_LOG_PERIOD_MS)) {
            ESP_LOGI(
                TAG,
                "current=%dmV model=%dmV %.0fg zero=%dmV",
                voltage_mv,
                fsr_sensor_get_model_mv(filtered_mv),
                grams,
                fsr_sensor_get_zero_mv());
            last_log_tick = now;
        }

        fsr_monitor_delay();
    }
}

esp_err_t fsr_monitor_start(void)
{
    if (s_task != NULL) {
        return ESP_OK;
    }

    esp_err_t error = fsr_sensor_init();
    if (error != ESP_OK) {
        return error;
    }

    s_peak_reset_tick = xTaskGetTickCount();
    s_peak_tick = s_peak_reset_tick;

    if (xTaskCreate(
            fsr_monitor_task,
            "fsr_monitor",
            FSR_MONITOR_TASK_STACK,
            NULL,
            FSR_MONITOR_TASK_PRIORITY,
            &s_task) != pdPASS) {
        s_task = NULL;
        return ESP_ERR_NO_MEM;
    }

    return ESP_OK;
}

esp_err_t fsr_monitor_capture_zero(void)
{
    return fsr_sensor_capture_zero();
}

int fsr_monitor_get_zero_mv(void)
{
    return fsr_sensor_get_zero_mv();
}

esp_err_t fsr_monitor_set_adc_offset_mv(int offset_mv)
{
    return fsr_sensor_set_adc_offset_mv(offset_mv);
}

float fsr_monitor_get_adc_offset_mv(void)
{
    return fsr_sensor_get_adc_offset_mv();
}

void fsr_monitor_set_periodic_logging(bool enabled)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_periodic_logging = enabled;
    taskEXIT_CRITICAL(&s_state_lock);
}

void fsr_monitor_reset_peak(void)
{
    taskENTER_CRITICAL(&s_state_lock);
    s_peak_mv = 0;
    s_peak_grams = 0.0f;
    s_peak_reset_tick = xTaskGetTickCount();
    s_peak_tick = s_peak_reset_tick;
    taskEXIT_CRITICAL(&s_state_lock);
}

int fsr_monitor_get_current_mv(void)
{
    int voltage_mv;
    taskENTER_CRITICAL(&s_state_lock);
    voltage_mv = s_current_mv;
    taskEXIT_CRITICAL(&s_state_lock);
    return voltage_mv;
}

//2026.10.14 新增：返回校正到万用表空载基准后的AO电压
int fsr_monitor_get_model_mv(void)
{
    int filtered_mv;
    taskENTER_CRITICAL(&s_state_lock);
    filtered_mv = s_current_filtered_mv;
    taskEXIT_CRITICAL(&s_state_lock);
    return fsr_sensor_get_model_mv(filtered_mv);
}

//2026.10.20 新增：原子读取同一次监测更新的电压和克重
void fsr_monitor_get_current_snapshot(
    int *raw_mv,
    int *model_mv,
    float *grams)
{
    taskENTER_CRITICAL(&s_state_lock);
    int current_mv = s_current_mv;
    int filtered_mv = s_current_filtered_mv;
    float current_grams = s_current_grams;
    taskEXIT_CRITICAL(&s_state_lock);

    if (raw_mv != NULL) {
        *raw_mv = current_mv;
    }
    if (model_mv != NULL) {
        *model_mv = fsr_sensor_get_model_mv(filtered_mv);
    }
    if (grams != NULL) {
        *grams = current_grams;
    }
}

float fsr_monitor_get_current_grams(void)
{
    float grams;
    taskENTER_CRITICAL(&s_state_lock);
    grams = s_current_grams;
    taskEXIT_CRITICAL(&s_state_lock);
    return grams;
}

int fsr_monitor_get_peak_mv(void)
{
    int voltage_mv;
    taskENTER_CRITICAL(&s_state_lock);
    voltage_mv = s_peak_mv;
    taskEXIT_CRITICAL(&s_state_lock);
    return voltage_mv;
}

float fsr_monitor_get_peak_grams(void)
{
    float grams;
    taskENTER_CRITICAL(&s_state_lock);
    grams = s_peak_grams;
    taskEXIT_CRITICAL(&s_state_lock);
    return grams;
}

unsigned int fsr_monitor_get_peak_time_ms(void)
{
    TickType_t reset_tick;
    TickType_t peak_tick;

    taskENTER_CRITICAL(&s_state_lock);
    reset_tick = s_peak_reset_tick;
    peak_tick = s_peak_tick;
    taskEXIT_CRITICAL(&s_state_lock);

    return (unsigned int)(
        (peak_tick - reset_tick) * portTICK_PERIOD_MS);
}

//2026.10.09 新增：获取平均压力值的函数
esp_err_t fsr_monitor_read_average_grams(
    int sample_count,
    int interval_ms,
    float *average_grams)
{
    if (sample_count <= 0 || interval_ms < 0 || average_grams == NULL) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    float sum_g = 0.0f;
    for (int index = 0; index < sample_count; ++index) 
    {
        sum_g += fsr_monitor_get_current_grams();

        if (index + 1 < sample_count && interval_ms > 0) 
        {
            TickType_t delay_ticks = pdMS_TO_TICKS(interval_ms);
            if (delay_ticks == 0) 
            {
                delay_ticks = 1;
            }
            vTaskDelay(delay_ticks);
        }
    }

    *average_grams = sum_g / (float)sample_count;
    return ESP_OK;
}
