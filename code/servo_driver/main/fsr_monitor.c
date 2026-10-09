#include "fsr_monitor.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "fsr_sensor.h"

#define FSR_MONITOR_TASK_STACK      3072
#define FSR_MONITOR_TASK_PRIORITY   4
#define FSR_MONITOR_PERIOD_MS       2
#define FSR_MONITOR_LOG_PERIOD_MS   500

static const char *TAG = "fsr_monitor";

static portMUX_TYPE s_state_lock = portMUX_INITIALIZER_UNLOCKED;
static TaskHandle_t s_task;
static int s_current_mv;
static float s_current_grams;
static int s_peak_mv;
static float s_peak_grams;
static TickType_t s_peak_reset_tick;
static TickType_t s_peak_tick;
static bool s_periodic_logging = true;

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

        float grams = fsr_sensor_mv_to_grams(voltage_mv);
        TickType_t now = xTaskGetTickCount();

        taskENTER_CRITICAL(&s_state_lock);
        s_current_mv = voltage_mv;
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
                "current=%dmV %.0fg zero=%dmV",
                voltage_mv,
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
