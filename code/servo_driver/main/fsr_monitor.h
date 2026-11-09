#ifndef FSR_MONITOR_H
#define FSR_MONITOR_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t fsr_monitor_start(void);
esp_err_t fsr_monitor_capture_zero(void);

//2026.10.09 新增：获取平均压力值的函数
esp_err_t fsr_monitor_read_average_grams(int sample_count,int interval_ms,float *average_grams);

int fsr_monitor_get_zero_mv(void);
//2026.10.21 新增：设置和读取人工标定的ADC偏移
esp_err_t fsr_monitor_set_adc_offset_mv(int offset_mv);
float fsr_monitor_get_adc_offset_mv(void);
void fsr_monitor_set_periodic_logging(bool enabled);
void fsr_monitor_reset_peak(void);
int fsr_monitor_get_current_mv(void);
//2026.10.21 修改：获取5点中值滤波并校正后的AO电压
int fsr_monitor_get_model_mv(void);
//2026.10.21 修改：原子读取原始电压、滤波后模型电压和克重
void fsr_monitor_get_current_snapshot(
    int *raw_mv,
    int *model_mv,
    float *grams);
float fsr_monitor_get_current_grams(void);
int fsr_monitor_get_peak_mv(void);
float fsr_monitor_get_peak_grams(void);
unsigned int fsr_monitor_get_peak_time_ms(void);

#endif
