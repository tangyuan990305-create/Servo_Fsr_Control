#ifndef FSR_SENSOR_H
#define FSR_SENSOR_H

#include "esp_err.h"

esp_err_t fsr_sensor_init(void);
esp_err_t fsr_sensor_read_raw(int *raw);
esp_err_t fsr_sensor_read_mv(int *voltage_mv);
esp_err_t fsr_sensor_capture_zero(void);
int fsr_sensor_get_zero_mv(void);
//2026.10.21 新增：显式设置ADC偏移，避免零点采集覆盖人工标定值
esp_err_t fsr_sensor_set_adc_offset_mv(int offset_mv);
float fsr_sensor_get_adc_offset_mv(void);
//2026.10.14 新增：获取校正到万用表空载基准后的AO电压
int fsr_sensor_get_model_mv(int voltage_mv);
float fsr_sensor_mv_to_grams(int voltage_mv);

#endif
