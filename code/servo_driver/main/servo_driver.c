#include "at_parser.h"
#include "drv_oled.h"
#include "fsr_monitor.h"
#include "servo_task.h"
#include "system_commands.h"
#include "uart_receiver.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define STARTUP_SERVO_ZERO_DELAY_MS 1500

static const char *TAG = "oled_hello";

void app_main(void)
{
    ESP_ERROR_CHECK(system_commands_init());
    ESP_ERROR_CHECK(fsr_monitor_start());

    ESP_LOGI(TAG, "Initializing OLED");
    ESP_ERROR_CHECK(drv_oled_init());

    drv_oled_clear();
    drv_oled_draw_text(34, 8, "HELLO", 2);
    ESP_ERROR_CHECK(drv_oled_refresh());

    ESP_LOGI(TAG, "HELLO is displayed");

    ESP_ERROR_CHECK(servo_task_start());

    vTaskDelay(pdMS_TO_TICKS(STARTUP_SERVO_ZERO_DELAY_MS));
    ESP_ERROR_CHECK(servo_set_angle(1, 0));
    ESP_ERROR_CHECK(servo_set_angle(2, 0));

    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(fsr_monitor_capture_zero());

    //2026.10.12 修改：启动回零和FSR零点采集完成后再接收AT命令，
    //避免启动阶段的命令被后续回零动作覆盖
    QueueHandle_t line_queue;
    ESP_ERROR_CHECK(uart_receiver_start(&line_queue));
    ESP_ERROR_CHECK(at_parser_start(
        line_queue,
        servo_set_id,
        servo_set_angle,
        servo_read_ids));

    ESP_LOGI(
        TAG,
        "AT commands ready: AT+SEED=<ID>, AT+ACTION=<ID>,<ANGLE>, AT+READID");
}
