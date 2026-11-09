#ifndef SERVO_TASK_H
#define SERVO_TASK_H

#include <stdbool.h>

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

//2026.10.09 修改：原注释为“根据压力设置按压角度”，
//实际接口用于保存压力闭环的目标克重。
//2026.10.09 新增：设置舵机压力闭环目标克重
esp_err_t servo_set_press_force(int servo_id, float grams);
//2026.10.09 修改：原注释含义不准确，实际返回压力闭环目标克重。
//2026.10.09 新增：获取舵机压力闭环目标克重
esp_err_t servo_get_press_force_setting(int servo_id, float *grams);
//2026.10.09 新增：获取舵机当前按压力，单位g
float servo_get_press_force(int servo_id);

//2026.10.10 新增：保存首次标定得到的最小压力对应角度
esp_err_t servo_set_press_min_angle(int servo_id, float angle_degrees);
//2026.10.10 新增：读取最小压力对应角度，未标定时返回ESP_ERR_NOT_FOUND
esp_err_t servo_get_press_min_angle_setting(
    int servo_id,
    float *angle_degrees);
//2026.10.10 新增：判断舵机是否已经完成最小压力角度标定
bool servo_has_press_min_angle(int servo_id);

int servo_get_active_id(void);
float servo_get_press_angle(int servo_id);
float servo_get_active_press_angle(void);

#endif
