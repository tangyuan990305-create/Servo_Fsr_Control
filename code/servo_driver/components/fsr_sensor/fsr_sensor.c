#include "fsr_sensor.h"

#include <stdbool.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define FSR_ADC_UNIT          ADC_UNIT_1
#define FSR_ADC_CHANNEL       ADC_CHANNEL_3
#define FSR_ADC_ATTEN         ADC_ATTEN_DB_6
#define FSR_ADC_BITWIDTH      ADC_BITWIDTH_DEFAULT
#define FSR_READ_SAMPLES      4
#define FSR_ZERO_SAMPLES      100
#define FSR_DEFAULT_ZERO_MV   120
#define FSR_FULL_SCALE_MV     3300
#define FSR_WEIGHT_MAX_G      20000.0f
#define FSR_DEADBAND_G        50.0f
#define FSR_LOCK_TIMEOUT_MS   100

static const char *TAG = "fsr_sensor";

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_calibration;
static bool s_calibrated;
static SemaphoreHandle_t s_lock;
static volatile int s_zero_mv = FSR_DEFAULT_ZERO_MV;

static bool fsr_sensor_adc_calibration_init(
    adc_unit_t unit,
    adc_channel_t channel,
    adc_atten_t atten,
    adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t error = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    adc_cali_curve_fitting_config_t curve_config = {
        .unit_id = unit,
        .chan = channel,
        .atten = atten,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    error = adc_cali_create_scheme_curve_fitting(
        &curve_config,
        &handle);
    calibrated = error == ESP_OK;
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        adc_cali_line_fitting_config_t line_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        error = adc_cali_create_scheme_line_fitting(
            &line_config,
            &handle);
        calibrated = error == ESP_OK;
    }
#endif

    *out_handle = handle;
    return calibrated;
}

static esp_err_t fsr_read_average_unlocked(
    int sample_count,
    int *raw_average)
{
    if (s_adc == NULL || raw_average == NULL || sample_count <= 0) {
        return ESP_ERR_INVALID_STATE;
    }

    int raw = 0;
    int raw_sum = 0;
    for (int index = 0; index < sample_count; ++index) {
        esp_err_t error = adc_oneshot_read(
            s_adc,
            FSR_ADC_CHANNEL,
            &raw);
        if (error != ESP_OK) {
            return error;
        }
        raw_sum += raw;
    }

    *raw_average = raw_sum / sample_count;
    return ESP_OK;
}

static esp_err_t fsr_take_lock(void)
{
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return xSemaphoreTake(
               s_lock,
               pdMS_TO_TICKS(FSR_LOCK_TIMEOUT_MS)) == pdTRUE
        ? ESP_OK
        : ESP_ERR_TIMEOUT;
}

static void fsr_give_lock(void)
{
    xSemaphoreGive(s_lock);
}

esp_err_t fsr_sensor_init(void)
{
    if (s_adc != NULL) {
        return ESP_OK;
    }

    s_lock = xSemaphoreCreateMutex();
    if (s_lock == NULL) {
        return ESP_ERR_NO_MEM;
    }

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = FSR_ADC_UNIT,
    };
    esp_err_t error = adc_oneshot_new_unit(
        &unit_config,
        &s_adc);
    if (error != ESP_OK) {
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return error;
    }

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = FSR_ADC_ATTEN,
        .bitwidth = FSR_ADC_BITWIDTH,
    };
    error = adc_oneshot_config_channel(
        s_adc,
        FSR_ADC_CHANNEL,
        &channel_config);
    if (error != ESP_OK) {
        adc_oneshot_del_unit(s_adc);
        s_adc = NULL;
        vSemaphoreDelete(s_lock);
        s_lock = NULL;
        return error;
    }

    s_calibrated = fsr_sensor_adc_calibration_init(
        FSR_ADC_UNIT,
        FSR_ADC_CHANNEL,
        FSR_ADC_ATTEN,
        &s_calibration);

    ESP_LOGI(
        TAG,
        "ADC ready: GPIO39 / ADC1_CHANNEL_3, calibration=%s",
        s_calibrated ? "yes" : "no");
    return ESP_OK;
}

esp_err_t fsr_sensor_read_raw(int *raw)
{
    if (raw == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = fsr_take_lock();
    if (error != ESP_OK) {
        return error;
    }

    error = fsr_read_average_unlocked(
        FSR_READ_SAMPLES,
        raw);
    fsr_give_lock();
    return error;
}

esp_err_t fsr_sensor_read_mv(int *voltage_mv)
{
    if (voltage_mv == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t error = fsr_take_lock();
    if (error != ESP_OK) {
        return error;
    }

    int raw = 0;
    error = fsr_read_average_unlocked(
        FSR_READ_SAMPLES,
        &raw);
    fsr_give_lock();
    if (error != ESP_OK) {
        return error;
    }

    if (s_calibrated) {
        error = adc_cali_raw_to_voltage(
            s_calibration,
            raw,
            voltage_mv);
        if (error == ESP_OK) {
            return ESP_OK;
        }
    }

    *voltage_mv = raw * 3300 / 4095;
    return ESP_OK;
}

esp_err_t fsr_sensor_capture_zero(void)
{
    int voltage_sum = 0;

    for (int index = 0; index < FSR_ZERO_SAMPLES; ++index) {
        int voltage_mv = 0;
        esp_err_t error = fsr_sensor_read_mv(&voltage_mv);
        if (error != ESP_OK) {
            return error;
        }
        voltage_sum += voltage_mv;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    s_zero_mv = voltage_sum / FSR_ZERO_SAMPLES;
    ESP_LOGI(TAG, "Zero captured: %dmV", s_zero_mv);
    return ESP_OK;
}

int fsr_sensor_get_zero_mv(void)
{
    return s_zero_mv;
}

float fsr_sensor_mv_to_grams(int voltage_mv)
{
    const float zero_mv = (float)s_zero_mv;
    const float span_mv = FSR_FULL_SCALE_MV - zero_mv;
    if (span_mv <= 0.0f) {
        return 0.0f;
    }

    float grams =
        ((float)voltage_mv - zero_mv) *
        FSR_WEIGHT_MAX_G / span_mv;

    if (grams < FSR_DEADBAND_G) {
        return 0.0f;
    }
    if (grams > FSR_WEIGHT_MAX_G) {
        return FSR_WEIGHT_MAX_G;
    }
    return grams;
}
