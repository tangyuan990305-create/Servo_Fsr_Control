#ifndef FSR_SENSOR_H
#define FSR_SENSOR_H

#include "esp_err.h"

esp_err_t fsr_sensor_init(void);
esp_err_t fsr_sensor_read_raw(int *raw);
esp_err_t fsr_sensor_read_mv(int *voltage_mv);
esp_err_t fsr_sensor_capture_zero(void);
int fsr_sensor_get_zero_mv(void);
float fsr_sensor_mv_to_grams(int voltage_mv);

#endif
