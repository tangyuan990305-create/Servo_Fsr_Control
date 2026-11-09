#include "fsr_sensor.h"

#include <stdbool.h>
#include <stddef.h>

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
//2026.10.21 修改：启动和机械调整后的零点存在漂移，
//延长稳定等待及平均窗口，避免捕获瞬态零点。
#define FSR_ZERO_SETTLE_MS    2000
#define FSR_ZERO_SAMPLES      1000
#define FSR_DEFAULT_ZERO_MV   120
#define FSR_LOCK_TIMEOUT_MS   100
//2026.10.14 新增：按万用表空载基准、AO-RES=131.7kΩ和
//IMS-C04A 1KG规格书建立50g~2kg参考模型。
//2026.10.21 修改：按当前装配空载106mV更新基准，ADC偏移由人工标定
//2026.11.09 修改：按当前硬件更新万用表空载基准
#define FSR_AO_DMM_ZERO_MV    100.6f
//2026.11.09 修改：按当前ADC与万用表空载差值校准
#define FSR_ADC_OFFSET_MV_DEFAULT 22.0f
//2026.11.09 修改：按当前实测AO-RES更新压力换算
#define FSR_AO_RES_KOHM       91.6f
#define FSR_MODEL_MIN_G       50.0f
#define FSR_MODEL_MAX_G       2000.0f

typedef struct {
    float grams;
    float resistance_kohm;
} fsr_force_point_t;

static const fsr_force_point_t s_force_points[] = {
    {50.0f, 280.0f},
    {100.0f, 141.0f},
    {200.0f, 68.0f},
    {500.0f, 22.0f},
    {1000.0f, 10.0f},
    {2000.0f, 4.2f},
};

static const char *TAG = "fsr_sensor";

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t s_calibration;
static bool s_calibrated;
static SemaphoreHandle_t s_lock;
static volatile int s_zero_mv = FSR_DEFAULT_ZERO_MV;
static volatile float s_adc_offset_mv = FSR_ADC_OFFSET_MV_DEFAULT;

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

    //2026.10.21 新增：先等待输入稳定，再开始零点平均
    vTaskDelay(pdMS_TO_TICKS(FSR_ZERO_SETTLE_MS));

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
    //2026.10.21 修改：零点采集只更新显示零点，
    //ADC偏移由人工标定值决定，避免短时漂移覆盖。
    ESP_LOGI(
        TAG,
        "Zero captured: %dmV, ADC offset: %.1fmV",
        s_zero_mv,
        s_adc_offset_mv);
    return ESP_OK;
}

int fsr_sensor_get_zero_mv(void)
{
    return s_zero_mv;
}

//2026.10.21 新增：保存人工标定的ADC偏移
esp_err_t fsr_sensor_set_adc_offset_mv(int offset_mv)
{
    if (offset_mv < 0 || offset_mv > 1000) {
        return ESP_ERR_INVALID_ARG;
    }
    s_adc_offset_mv = (float)offset_mv;
    return ESP_OK;
}

float fsr_sensor_get_adc_offset_mv(void)
{
    return s_adc_offset_mv;
}

//2026.10.14 新增：把ESP32 ADC读数校正到万用表空载基准
int fsr_sensor_get_model_mv(int voltage_mv)
{
    float offset_mv = s_adc_offset_mv;
    int rounded_offset_mv = offset_mv >= 0.0f
        ? (int)(offset_mv + 0.5f)
        : (int)(offset_mv - 0.5f);
    return voltage_mv - rounded_offset_mv;
}

float fsr_sensor_mv_to_grams(int voltage_mv)
{
    const float model_mv = (float)fsr_sensor_get_model_mv(voltage_mv);
    const float delta_mv = model_mv - FSR_AO_DMM_ZERO_MV;
    if (delta_mv <= 0.0f) {
        return 0.0f;
    }

    // U_AO - 106mV = 0.1V * R_AO-RES / R_FSR
    const float conductance =
        delta_mv / (100.0f * FSR_AO_RES_KOHM);

    const float first_conductance =
        1.0f / s_force_points[0].resistance_kohm;
    if (conductance < first_conductance) {
        return 0.0f;
    }

    const size_t point_count =
        sizeof(s_force_points) / sizeof(s_force_points[0]);
    for (size_t index = 1; index < point_count; ++index) {
        const float lower_conductance =
            1.0f / s_force_points[index - 1].resistance_kohm;
        const float upper_conductance =
            1.0f / s_force_points[index].resistance_kohm;
        if (conductance <= upper_conductance ||
            index + 1 == point_count) {
            if (conductance >= upper_conductance) {
                return FSR_MODEL_MAX_G;
            }

            const float ratio =
                (conductance - lower_conductance) /
                (upper_conductance - lower_conductance);
            return s_force_points[index - 1].grams +
                ratio * (s_force_points[index].grams -
                         s_force_points[index - 1].grams);
        }
    }

    return FSR_MODEL_MAX_G;
}
