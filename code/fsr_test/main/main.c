#include <stdbool.h>
#include <stdio.h>

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define FSR_ADC_UNIT         ADC_UNIT_1
/* GPIO39 is the fifth physical pad from the antenna end. */
#define FSR_ADC_CHANNEL      ADC_CHANNEL_3
/*
 * Use 6 dB while testing the low-voltage end near 0.1 V.
 * Switch back to ADC_ATTEN_DB_12 for the full 0-3.3 V range.
 */
#define FSR_ADC_ATTEN        ADC_ATTEN_DB_6
#define FSR_ADC_BITWIDTH     ADC_BITWIDTH_DEFAULT
#define FSR_SAMPLE_COUNT     32
#define FSR_LOG_PERIOD_MS    500

#define FSR_VOLTAGE_ZERO_MV  120.0f
#define FSR_VOLTAGE_FULL_MV  3300.0f
#define FSR_WEIGHT_MAX_G     20000.0f
#define FSR_DEADBAND_G       50.0f

static const char *TAG = "fsr_test";

static bool fsr_adc_calibration_init(
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
    if (calibrated) {
        ESP_LOGI(TAG, "ADC calibration enabled");
    } else {
        ESP_LOGW(TAG, "ADC calibration unavailable; using raw scaling");
    }
    return calibrated;
}

static int fsr_read_average(
    adc_oneshot_unit_handle_t adc_handle)
{
    int raw = 0;
    int raw_sum = 0;

    for (int index = 0; index < FSR_SAMPLE_COUNT; ++index) {
        ESP_ERROR_CHECK(adc_oneshot_read(
            adc_handle,
            FSR_ADC_CHANNEL,
            &raw));
        raw_sum += raw;
    }

    return raw_sum / FSR_SAMPLE_COUNT;
}

static float fsr_estimate_grams(int voltage_mv)
{
    const float voltage_span_mv =
        FSR_VOLTAGE_FULL_MV - FSR_VOLTAGE_ZERO_MV;
    float grams =
        ((float)voltage_mv - FSR_VOLTAGE_ZERO_MV) *
        FSR_WEIGHT_MAX_G / voltage_span_mv;

    if (grams < FSR_DEADBAND_G) {
        return 0.0f;
    }
    if (grams > FSR_WEIGHT_MAX_G) {
        return FSR_WEIGHT_MAX_G;
    }
    return grams;
}

void app_main(void)
{
    adc_oneshot_unit_handle_t adc_handle = NULL;
    adc_cali_handle_t calibration_handle = NULL;

    adc_oneshot_unit_init_cfg_t unit_config = {
        .unit_id = FSR_ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(
        &unit_config,
        &adc_handle));

    adc_oneshot_chan_cfg_t channel_config = {
        .atten = FSR_ADC_ATTEN,
        .bitwidth = FSR_ADC_BITWIDTH,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(
        adc_handle,
        FSR_ADC_CHANNEL,
        &channel_config));

    bool calibrated = fsr_adc_calibration_init(
        FSR_ADC_UNIT,
        FSR_ADC_CHANNEL,
        FSR_ADC_ATTEN,
        &calibration_handle);

    ESP_LOGI(TAG, "FSR test ready: GPIO39 / ADC1_CHANNEL_3");

    while (true) {
        int raw_average = fsr_read_average(adc_handle);
        int voltage_mv = raw_average * 3300 / 4095;

        if (calibrated) {
            ESP_ERROR_CHECK(adc_cali_raw_to_voltage(
                calibration_handle,
                raw_average,
                &voltage_mv));
        }

        float estimated_grams =
            fsr_estimate_grams(voltage_mv);

        ESP_LOGI(
            TAG,
            "raw=%d voltage=%dmV estimated=%.0fg",
            raw_average,
            voltage_mv,
            estimated_grams);

        vTaskDelay(pdMS_TO_TICKS(FSR_LOG_PERIOD_MS));
    }
}
