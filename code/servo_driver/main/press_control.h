//2026.10.09 新增：防止头文件被重复包含
#ifndef PRESS_CONTROL_H
#define PRESS_CONTROL_H

#include <stdbool.h>

#include "esp_err.h"

//2026.10.09 新增：压力闭环控制的输入参数
typedef struct {
    int servo_id;
    //2026.10.10 修改：使用按键允许的最小和最大压力，
    //停止点靠近最小压力并叠加少量安全裕量
    float min_g;
    float max_g;
    //2026.10.12 新增：允许的最小峰值压力，0表示不启用峰值下限
    float peak_min_g;
    float margin_g;
    float max_angle_deg;
    int hold_ms;
    int timeout_ms;
} press_request_t;

//2026.10.09 新增：压力闭环控制的执行结果
typedef struct {
    float target_g;
    float reached_g;
    float peak_g;
    float final_angle_deg;
    int elapsed_ms;
    bool target_reached;
} press_result_t;

//2026.10.10 新增：一次2000ms固定角度压力窗口的统计结果
typedef struct {
    float average_g;
    float median_g;
    float peak_g;
    float max_moving_average_g;
    float final_average_g;
    int sample_count;
} press_measurement_t;

//2026.10.09 新增：执行一次压力闭环按压
esp_err_t press_control_execute(
    const press_request_t *request,
    press_result_t *result);

//2026.10.10 新增：从最大安全角回退，标定最小稳定压力角度并保存
esp_err_t press_control_calibrate(
    const press_request_t *request,
    press_result_t *result);

//2026.10.10 新增：固定角度采集一次压力窗口，不标定也不修改保存角度
esp_err_t press_control_measure_fixed(
    int servo_id,
    float angle_degrees,
    press_measurement_t *measurement);

//2026.10.20 新增：控制固定角度测试的逐点压力输出
void press_control_set_trace_enabled(bool enabled);
bool press_control_trace_is_enabled(void);
void press_control_set_trace_cycle(int cycle_index);

#endif
