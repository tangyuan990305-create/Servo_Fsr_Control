#ifndef SERVO_TASK_H
#define SERVO_TASK_H

#include "esp_err.h"

esp_err_t servo_task_start(void);
esp_err_t servo_button_start(void);
esp_err_t servo_set_id(int new_id);
esp_err_t servo_set_angle(int servo_id, float angle_degrees);
esp_err_t servo_read_ids(void);
esp_err_t servo_read_current(int servo_id, int16_t *current);
esp_err_t servo_read_position(int servo_id, uint16_t *position);
esp_err_t servo_set_press_angle(int servo_id, float angle_degrees);
esp_err_t servo_get_press_angle_setting(int servo_id, float *angle_degrees);
int servo_get_active_id(void);
float servo_get_press_angle(int servo_id);
float servo_get_active_press_angle(void);

#endif
