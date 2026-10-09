#ifndef FSR_MONITOR_H
#define FSR_MONITOR_H

#include <stdbool.h>

#include "esp_err.h"

esp_err_t fsr_monitor_start(void);
esp_err_t fsr_monitor_capture_zero(void);
int fsr_monitor_get_zero_mv(void);
void fsr_monitor_set_periodic_logging(bool enabled);
void fsr_monitor_reset_peak(void);
int fsr_monitor_get_current_mv(void);
float fsr_monitor_get_current_grams(void);
int fsr_monitor_get_peak_mv(void);
float fsr_monitor_get_peak_grams(void);
unsigned int fsr_monitor_get_peak_time_ms(void);

#endif
