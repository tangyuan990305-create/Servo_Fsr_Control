#include "servo_task.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "driver/gpio.h"
#include "driver/uart.h"
#include "drv_oled.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
//2026.10.09 修改：servo_get_press_force 改为返回各舵机保存的目标克重，
//与 servo_get_press_angle 保持同一语义，不再读取 FSR 实时值，
//因此不再需要 fsr_monitor.h
//#include "fsr_monitor.h"
#include "nvs.h"

#define SERVO_UART          UART_NUM_1
#define SERVO_RX_GPIO       18
#define SERVO_TX_GPIO       19
#define SERVO_BAUD_RATE     1000000

#define SERVO_BROADCAST_ID  0xFE
#define SERVO_ZERO_POSITION 2047
#define SERVO_POSITION_SPAN 2047
#define SERVO_SPEED         1500
#define SERVO_ACCELERATION  50
#define SERVO_REPLY_TIMEOUT_MS 20
#define SERVO_PRESENT_POSITION_ADDR 56
#define SERVO_PRESENT_CURRENT_ADDR 69
#define SERVO_BUTTON_GPIO      GPIO_NUM_0
#define SERVO_BUTTON_TASK_STACK 2048
#define SERVO_BUTTON_TASK_PRIORITY 4
#define SERVO_BUTTON_DEBOUNCE_MS 30
#define SERVO_PRESS_ANGLE_1_DEFAULT (-75.0f)
#define SERVO_PRESS_ANGLE_2_DEFAULT 75.0f
#define SERVO_PRESS_ANGLE_MIN (-180.0f)
#define SERVO_PRESS_ANGLE_MAX 180.0f
#define SERVO_PRESS_FORCE_1_DEFAULT 222.8f
#define SERVO_PRESS_FORCE_2_DEFAULT 306.3f
#define SERVO_PRESS_FORCE_MIN_G 0.1f
#define SERVO_PRESS_FORCE_MAX_G 20000.0f
#define SERVO_PRESS_FORCE_SCALE 10.0f

static const char *TAG = "servo_task";
static SemaphoreHandle_t s_servo_mutex;
static volatile int s_active_servo_id = 1;
static bool s_button_started;
static float s_press_angle_1 = SERVO_PRESS_ANGLE_1_DEFAULT;
static float s_press_angle_2 = SERVO_PRESS_ANGLE_2_DEFAULT;
static float s_press_force_1 = SERVO_PRESS_FORCE_1_DEFAULT;
static float s_press_force_2 = SERVO_PRESS_FORCE_2_DEFAULT;
//2026.10.10 新增：首次标定得到的最小压力角度及有效标志
static float s_press_min_angle_1;
static float s_press_min_angle_2;
static bool s_press_min_angle_1_valid;
static bool s_press_min_angle_2_valid;

static void servo_display_active_mode(void)
{
    const char *text = "HELLO";

    drv_oled_clear();
    drv_oled_draw_text(28, 8, text, 2);
    esp_err_t error = drv_oled_refresh();
    if (error != ESP_OK) {
        ESP_LOGE(
            TAG,
            "Failed to display servo mode: %s",
            esp_err_to_name(error));
    }
}

int servo_get_active_id(void)
{
    return s_active_servo_id;
}

static const char *servo_press_angle_key(int servo_id)
{
    return servo_id == 1 ? "press1_x10" : "press2_x10";
}

static const char *servo_press_force_key(int servo_id)
{
    return servo_id == 1 ? "press1_g_x10" : "press2_g_x10";
}

//2026.10.10 新增：最小压力角度使用独立的短键名，避免覆盖最大安全角度
static const char *servo_press_min_angle_key(int servo_id)
{
    return servo_id == 1 ? "minang1_x10" : "minang2_x10";
}

static float servo_get_default_press_angle(int servo_id)
{
    return servo_id == 1
        ? SERVO_PRESS_ANGLE_1_DEFAULT
        : SERVO_PRESS_ANGLE_2_DEFAULT;
}

static float servo_get_default_press_force(int servo_id)
{
    return servo_id == 1
        ? SERVO_PRESS_FORCE_1_DEFAULT
        : SERVO_PRESS_FORCE_2_DEFAULT;
}

static esp_err_t servo_load_press_angle(int servo_id, float *angle_degrees)
{
    if (angle_degrees == NULL || servo_id < 1 || servo_id > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t error = nvs_open("servo_cfg", NVS_READONLY, &handle);
    if (error == ESP_ERR_NVS_NOT_FOUND) {
        *angle_degrees = servo_get_default_press_angle(servo_id);
        return ESP_OK;
    }
    if (error != ESP_OK) {
        return error;
    }

    int32_t stored_angle_x10 = 0;
    error = nvs_get_i32(
        handle,
        servo_press_angle_key(servo_id),
        &stored_angle_x10);
    nvs_close(handle);

    if (error == ESP_ERR_NVS_NOT_FOUND) {
        *angle_degrees = servo_get_default_press_angle(servo_id);
        return ESP_OK;
    }
    if (error != ESP_OK) {
        return error;
    }

    *angle_degrees = (float)stored_angle_x10 / 10.0f;
    return ESP_OK;
}

//2026.10.09 新增：从NVS加载压力闭环目标克重
static esp_err_t servo_load_press_force(int servo_id, float *grams)
{
    if (grams == NULL || servo_id < 1 || servo_id > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    nvs_handle_t handle;
    esp_err_t error = nvs_open("servo_cfg", NVS_READONLY, &handle);
    if (error == ESP_ERR_NVS_NOT_FOUND) {
        *grams = servo_get_default_press_force(servo_id);
        return ESP_OK;
    }
    if (error != ESP_OK) {
        return error;
    }

    int32_t stored_force_x10 = 0;
    error = nvs_get_i32(
        handle,
        servo_press_force_key(servo_id),
        &stored_force_x10);
    nvs_close(handle);

    if (error == ESP_ERR_NVS_NOT_FOUND) {
        *grams = servo_get_default_press_force(servo_id);
        return ESP_OK;
    }
    if (error != ESP_OK) {
        return error;
    }

    *grams = (float)stored_force_x10 / SERVO_PRESS_FORCE_SCALE;
    return ESP_OK;
}

//2026.10.10 新增：从NVS加载最小压力对应角度
static esp_err_t servo_load_press_min_angle(
    int servo_id,
    float *angle_degrees,
    bool *valid)
{
    if (angle_degrees == NULL || valid == NULL ||
        servo_id < 1 || servo_id > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    *angle_degrees = 0.0f;
    *valid = false;

    nvs_handle_t handle;
    esp_err_t error = nvs_open("servo_cfg", NVS_READONLY, &handle);
    if (error == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (error != ESP_OK) {
        return error;
    }

    int32_t stored_angle_x10 = 0;
    error = nvs_get_i32(
        handle,
        servo_press_min_angle_key(servo_id),
        &stored_angle_x10);
    nvs_close(handle);

    if (error == ESP_ERR_NVS_NOT_FOUND) {
        return ESP_OK;
    }
    if (error != ESP_OK) {
        return error;
    }

    *angle_degrees = (float)stored_angle_x10 / 10.0f;
    *valid = true;
    return ESP_OK;
}

float servo_get_press_angle(int servo_id)
{
    return servo_id == 1 ? s_press_angle_1 : s_press_angle_2;
}

float servo_get_active_press_angle(void)
{
    return servo_get_press_angle(s_active_servo_id);
}

esp_err_t servo_get_press_angle_setting(int servo_id, float *angle_degrees)
{
    if (angle_degrees == NULL || servo_id < 1 || servo_id > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    *angle_degrees = servo_get_press_angle(servo_id);
    return ESP_OK;
}

esp_err_t servo_set_press_angle(int servo_id, float angle_degrees)
{
    if (servo_id < 1 || servo_id > 2 ||
        angle_degrees < SERVO_PRESS_ANGLE_MIN ||
        angle_degrees > SERVO_PRESS_ANGLE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }

    if (servo_id == 1 && angle_degrees >= 15.0f) {
        ESP_LOGW(
            TAG,
            "Servo 1 press angle %.1f rejected: must be less than 15 degrees",
            angle_degrees);
        return ESP_ERR_INVALID_ARG;
    }
    if (servo_id == 2 && angle_degrees <= -15.0f) {
        ESP_LOGW(
            TAG,
            "Servo 2 press angle %.1f rejected: must be greater than -15 degrees",
            angle_degrees);
        return ESP_ERR_INVALID_ARG;
    }

    int32_t stored_angle_x10 = angle_degrees >= 0.0f
        ? (int32_t)(angle_degrees * 10.0f + 0.5f)
        : (int32_t)(angle_degrees * 10.0f - 0.5f);

    nvs_handle_t handle;
    esp_err_t error = nvs_open("servo_cfg", NVS_READWRITE, &handle);
    if (error != ESP_OK) {
        return error;
    }

    error = nvs_set_i32(
        handle,
        servo_press_angle_key(servo_id),
        stored_angle_x10);
    if (error == ESP_OK) {
        error = nvs_commit(handle);
    }
    nvs_close(handle);

    if (error == ESP_OK) {
        if (servo_id == 1) {
            s_press_angle_1 = angle_degrees;
        } else {
            s_press_angle_2 = angle_degrees;
        }
    }

    return error;
}

static void servo_button_task(void *argument)
{
    (void)argument;

    while (true) {
        if (gpio_get_level(SERVO_BUTTON_GPIO) != 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(SERVO_BUTTON_DEBOUNCE_MS));
        if (gpio_get_level(SERVO_BUTTON_GPIO) != 0) {
            continue;
        }

        s_active_servo_id =
            s_active_servo_id == 1 ? 2 : 1;
        servo_display_active_mode();
        ESP_LOGI(
            TAG,
        "Button selected servo %d, press angle %+.2f degrees",
            s_active_servo_id,
            servo_get_active_press_angle());

        while (gpio_get_level(SERVO_BUTTON_GPIO) == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        vTaskDelay(pdMS_TO_TICKS(SERVO_BUTTON_DEBOUNCE_MS));
    }
}

esp_err_t servo_button_start(void)
{
    if (s_button_started) {
        return ESP_ERR_INVALID_STATE;
    }

    const gpio_config_t config = {
        .pin_bit_mask = 1ULL << SERVO_BUTTON_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    esp_err_t error = gpio_config(&config);
    if (error != ESP_OK) {
        return error;
    }

    if (xTaskCreate(
            servo_button_task,
            "servo_button",
            SERVO_BUTTON_TASK_STACK,
            NULL,
            SERVO_BUTTON_TASK_PRIORITY,
            NULL) != pdPASS) {
        return ESP_ERR_NO_MEM;
    }

    s_button_started = true;
    servo_display_active_mode();
    ESP_LOGI(TAG, "Button selector ready: GPIO0, default servo 1");
    return ESP_OK;
}

static esp_err_t servo_write_byte(
    uint8_t id,
    uint8_t address,
    uint8_t value)
{
    uint8_t packet[8] = {
        0xFF, 0xFF,
        id,
        4,
        0x03,
        address,
        value,
        0
    };

    uint16_t sum = 0;
    for (int i = 2; i <= 6; ++i) {
        sum += packet[i];
    }
    packet[7] = (uint8_t)~sum;

    int written =
        uart_write_bytes(SERVO_UART, packet, sizeof(packet));
    if (written != sizeof(packet)) {
        return ESP_FAIL;
    }

    return uart_wait_tx_done(
        SERVO_UART, pdMS_TO_TICKS(100));
}

static esp_err_t servo_write_position(
    uint8_t id,
    uint16_t position)
{
    uint8_t packet[14] = {
        0xFF, 0xFF,
        id,
        10,
        0x03,
        41,
        SERVO_ACCELERATION,
        (uint8_t)(position & 0xFF),
        (uint8_t)(position >> 8),
        0, 0,
        (uint8_t)(SERVO_SPEED & 0xFF),
        (uint8_t)(SERVO_SPEED >> 8),
        0
    };

    uint16_t sum = 0;
    for (int i = 2; i <= 12; ++i) {
        sum += packet[i];
    }
    packet[13] = (uint8_t)~sum;

    int written =
        uart_write_bytes(SERVO_UART, packet, sizeof(packet));
    if (written != sizeof(packet)) {
        return ESP_FAIL;
    }

    return uart_wait_tx_done(
        SERVO_UART, pdMS_TO_TICKS(100));
}

static esp_err_t servo_send_instruction(
    uint8_t id,
    uint8_t instruction,
    const uint8_t *parameters,
    size_t parameter_count)
{
    if (parameter_count > 8) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t packet[14] = {
        0xFF,
        0xFF,
        id,
        (uint8_t)(parameter_count + 2),
        instruction
    };
    uint16_t sum = id + packet[3] + instruction;
    for (size_t i = 0; i < parameter_count; ++i) {
        packet[5 + i] = parameters[i];
        sum += parameters[i];
    }
    packet[5 + parameter_count] = (uint8_t)~sum;

    size_t packet_length = parameter_count + 6;
    int written = uart_write_bytes(
        SERVO_UART,
        packet,
        packet_length);
    if (written != (int)packet_length) {
        return ESP_FAIL;
    }
    return uart_wait_tx_done(
        SERVO_UART,
        pdMS_TO_TICKS(100));
}

static esp_err_t servo_receive_status(
    uint8_t expected_id,
    uint8_t *parameters,
    size_t parameter_capacity,
    size_t *parameter_count)
{
    uint8_t data[32];
    int received = uart_read_bytes(
        SERVO_UART,
        data,
        sizeof(data),
        pdMS_TO_TICKS(SERVO_REPLY_TIMEOUT_MS));
    if (received <= 0) {
        return ESP_ERR_TIMEOUT;
    }

    for (int start = 0; start + 6 <= received; ++start) {
        if (data[start] != 0xFF || data[start + 1] != 0xFF) {
            continue;
        }

        uint8_t id = data[start + 2];
        uint8_t length = data[start + 3];
        int packet_length = length + 4;
        if (length < 2 || start + packet_length > received ||
            id != expected_id) {
            continue;
        }

        uint16_t sum = 0;
        for (int i = start + 2;
             i < start + packet_length - 1;
             ++i) {
            sum += data[i];
        }
        uint8_t checksum = (uint8_t)~sum;
        if (checksum != data[start + packet_length - 1]) {
            continue;
        }
        if (data[start + 4] != 0) {
            return ESP_FAIL;
        }

        size_t count = length - 2;
        if (count > parameter_capacity) {
            return ESP_ERR_INVALID_SIZE;
        }
        for (size_t i = 0; i < count; ++i) {
            parameters[i] = data[start + 5 + i];
        }
        *parameter_count = count;
        return ESP_OK;
    }

    return ESP_ERR_TIMEOUT;
}

static esp_err_t servo_ping(uint8_t id)
{
    uart_flush_input(SERVO_UART);
    esp_err_t error =
        servo_send_instruction(id, 0x01, NULL, 0);
    if (error != ESP_OK) {
        return error;
    }

    uint8_t unused[1];
    size_t parameter_count;
    return servo_receive_status(
        id,
        unused,
        sizeof(unused),
        &parameter_count);
}

static esp_err_t servo_read_eeprom_id(
    uint8_t id,
    uint8_t *eeprom_id)
{
    const uint8_t read_parameters[] = {5, 1};

    uart_flush_input(SERVO_UART);
    esp_err_t error = servo_send_instruction(
        id,
        0x02,
        read_parameters,
        sizeof(read_parameters));
    if (error != ESP_OK) {
        return error;
    }

    size_t parameter_count;
    error = servo_receive_status(
        id,
        eeprom_id,
        1,
        &parameter_count);
    if (error != ESP_OK) {
        return error;
    }
    return parameter_count == 1
        ? ESP_OK
        : ESP_ERR_INVALID_RESPONSE;
}

static esp_err_t servo_read_register(
    int servo_id, uint8_t address, uint8_t *data, size_t length)
{
    if (servo_id < 1 || servo_id >= SERVO_BROADCAST_ID ||
        data == NULL || length == 0 || length > 8) {
        return ESP_ERR_INVALID_ARG;
    }
    const uint8_t parameters[] = {address, (uint8_t)length};
    uart_flush_input(SERVO_UART);
    esp_err_t error = servo_send_instruction(
        (uint8_t)servo_id, 0x02, parameters, sizeof(parameters));
    if (error != ESP_OK) return error;
    size_t count = 0;
    error = servo_receive_status((uint8_t)servo_id, data, length, &count);
    return error == ESP_OK && count == length ? ESP_OK :
        (error == ESP_OK ? ESP_ERR_INVALID_RESPONSE : error);
}

esp_err_t servo_read_current(int servo_id, int16_t *current)
{
    if (current == NULL || s_servo_mutex == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t data[2];
    xSemaphoreTake(s_servo_mutex, portMAX_DELAY);
    esp_err_t error = servo_read_register(
        servo_id, SERVO_PRESENT_CURRENT_ADDR, data, sizeof(data));
    xSemaphoreGive(s_servo_mutex);
    if (error == ESP_OK) *current = (int16_t)((uint16_t)data[0] | ((uint16_t)data[1] << 8));
    return error;
}

esp_err_t servo_read_position(int servo_id, uint16_t *position)
{
    if (position == NULL || s_servo_mutex == NULL) return ESP_ERR_INVALID_STATE;
    uint8_t data[2];
    xSemaphoreTake(s_servo_mutex, portMAX_DELAY);
    esp_err_t error = servo_read_register(
        servo_id, SERVO_PRESENT_POSITION_ADDR, data, sizeof(data));
    xSemaphoreGive(s_servo_mutex);
    if (error == ESP_OK) *position = (uint16_t)data[0] | ((uint16_t)data[1] << 8);
    return error;
}

esp_err_t servo_set_id(int new_id)
{
    if (new_id < 1 || new_id >= SERVO_BROADCAST_ID) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_servo_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_servo_mutex, portMAX_DELAY);

    ESP_LOGI(TAG, "Broadcast: unlock EEPROM");
    esp_err_t error =
        servo_write_byte(SERVO_BROADCAST_ID, 55, 0);
    vTaskDelay(pdMS_TO_TICKS(100));

    if (error == ESP_OK) {
        ESP_LOGI(
            TAG,
            "Broadcast: set connected servo ID to %d",
            new_id);
        error = servo_write_byte(
            SERVO_BROADCAST_ID,
            5,
            (uint8_t)new_id);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    if (error == ESP_OK) {
        ESP_LOGI(TAG, "Servo %d: lock EEPROM", new_id);
        error = servo_write_byte((uint8_t)new_id, 55, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    xSemaphoreGive(s_servo_mutex);
    return error;
}

esp_err_t servo_set_angle(int servo_id, float angle_degrees)
{
    if (servo_id < 1 || servo_id >= SERVO_BROADCAST_ID ||
        angle_degrees < -180.0f || angle_degrees > 180.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (servo_id == 1 && angle_degrees >= 15.0f) {
        ESP_LOGW(
            TAG,
            "Servo 1 angle %.1f rejected: must be less than 15 degrees",
            angle_degrees);
        return ESP_ERR_INVALID_ARG;
    }
    if (servo_id == 2 && angle_degrees <= -15.0f) {
        ESP_LOGW(
            TAG,
            "Servo 2 angle %.1f rejected: must be greater than -15 degrees",
            angle_degrees);
        return ESP_ERR_INVALID_ARG;
    }
    if (s_servo_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    float scaled = angle_degrees * SERVO_POSITION_SPAN / 180.0f;
    int offset = scaled >= 0.0f
        ? (int)(scaled + 0.5f)
        : (int)(scaled - 0.5f);
    uint16_t position =
        (uint16_t)(SERVO_ZERO_POSITION + offset);

    xSemaphoreTake(s_servo_mutex, portMAX_DELAY);
    ESP_LOGI(
        TAG,
        //2026.10.13 修改：记录0.05度固定测试的实际目标角度
        "Servo %d -> %.2f degrees (position %u)",
        servo_id,
        angle_degrees,
        position);
    esp_err_t error =
        servo_write_position((uint8_t)servo_id, position);
    xSemaphoreGive(s_servo_mutex);
    return error;
}

esp_err_t servo_read_ids(void)
{
    if (s_servo_mutex == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    xSemaphoreTake(s_servo_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "Scanning servo IDs 1-253...");

    int found = 0;
    for (int id = 1; id < SERVO_BROADCAST_ID; ++id) {
        if (servo_ping((uint8_t)id) != ESP_OK) {
            continue;
        }

        uint8_t eeprom_id;
        esp_err_t error =
            servo_read_eeprom_id((uint8_t)id, &eeprom_id);
        if (error == ESP_OK) {
            ESP_LOGI(
                TAG,
                "SERVO FOUND: response ID=%d, EEPROM ID=%u",
                id,
                eeprom_id);
        } else {
            ESP_LOGW(
                TAG,
                "SERVO FOUND: response ID=%d, EEPROM read failed: %s",
                id,
                esp_err_to_name(error));
        }
        ++found;
    }

    xSemaphoreGive(s_servo_mutex);
    return found > 0 ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t servo_task_start(void)
{
    if (s_servo_mutex != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    const uart_config_t config = {
        .baud_rate = SERVO_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };

    ESP_ERROR_CHECK(uart_param_config(SERVO_UART, &config));
    ESP_ERROR_CHECK(
        uart_set_pin(
            SERVO_UART,
            SERVO_TX_GPIO,
            SERVO_RX_GPIO,
            UART_PIN_NO_CHANGE,
            UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(
        uart_driver_install(SERVO_UART, 256, 0, 0, NULL, 0));

    s_servo_mutex = xSemaphoreCreateMutex();
    if (s_servo_mutex == NULL) {
        uart_driver_delete(SERVO_UART);
        return ESP_ERR_NO_MEM;
    }

    esp_err_t load_error =
        servo_load_press_angle(1, &s_press_angle_1);
    if (load_error != ESP_OK) {
        ESP_LOGW(
            TAG,
            "Load servo 1 press angle failed: %s",
            esp_err_to_name(load_error));
        s_press_angle_1 = SERVO_PRESS_ANGLE_1_DEFAULT;
    }

    load_error = servo_load_press_angle(2, &s_press_angle_2);
    if (load_error != ESP_OK) {
        ESP_LOGW(
            TAG,
            "Load servo 2 press angle failed: %s",
            esp_err_to_name(load_error));
        s_press_angle_2 = SERVO_PRESS_ANGLE_2_DEFAULT;
    }

    load_error = servo_load_press_force(1, &s_press_force_1);
    if (load_error != ESP_OK) {
        ESP_LOGW(
            TAG,
            "Load servo 1 press force failed: %s",
            esp_err_to_name(load_error));
        s_press_force_1 = SERVO_PRESS_FORCE_1_DEFAULT;
    }

    load_error = servo_load_press_force(2, &s_press_force_2);
    if (load_error != ESP_OK) {
        ESP_LOGW(
            TAG,
            "Load servo 2 press force failed: %s",
            esp_err_to_name(load_error));
        s_press_force_2 = SERVO_PRESS_FORCE_2_DEFAULT;
    }

    //2026.10.10 新增：加载最小压力角度，未标定时保持无效状态
    load_error = servo_load_press_min_angle(
        1,
        &s_press_min_angle_1,
        &s_press_min_angle_1_valid);
    if (load_error != ESP_OK) {
        ESP_LOGW(
            TAG,
            "Load servo 1 min press angle failed: %s",
            esp_err_to_name(load_error));
        s_press_min_angle_1 = 0.0f;
        s_press_min_angle_1_valid = false;
    }

    load_error = servo_load_press_min_angle(
        2,
        &s_press_min_angle_2,
        &s_press_min_angle_2_valid);
    if (load_error != ESP_OK) {
        ESP_LOGW(
            TAG,
            "Load servo 2 min press angle failed: %s",
            esp_err_to_name(load_error));
        s_press_min_angle_2 = 0.0f;
        s_press_min_angle_2_valid = false;
    }

    ESP_LOGI(
        TAG,
        "Press angles loaded: servo 1 %+.1f, servo 2 %+.1f degrees",
        s_press_angle_1,
        s_press_angle_2);
    ESP_LOGI(
        TAG,
        "Press forces loaded: servo 1 %.1fg, servo 2 %.1fg",
        s_press_force_1,
        s_press_force_2);
    ESP_LOGI(
        TAG,
        "Min press angles loaded: servo 1 %s%.1f, servo 2 %s%.1f",
        s_press_min_angle_1_valid ? "" : "invalid ",
        s_press_min_angle_1,
        s_press_min_angle_2_valid ? "" : "invalid ",
        s_press_min_angle_2);

    ESP_LOGI(TAG, "Servo UART ready for AT commands");
    return ESP_OK;
}


//2026.10.09 新增：保存舵机压力闭环目标克重
esp_err_t servo_set_press_force(int servo_id, float grams)
{
    if (servo_id < 1 || servo_id > 2 ||
        grams < SERVO_PRESS_FORCE_MIN_G ||
        grams > SERVO_PRESS_FORCE_MAX_G) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t stored_force_x10 =
        (int32_t)(grams * SERVO_PRESS_FORCE_SCALE + 0.5f);
    nvs_handle_t handle;
    esp_err_t error = nvs_open("servo_cfg", NVS_READWRITE, &handle);
    if (error != ESP_OK) 
    {
        return error;
    }

    error = nvs_set_i32(
        handle,
        servo_press_force_key(servo_id),
        stored_force_x10);
    if (error == ESP_OK) 
    {
        error = nvs_commit(handle);
    }
    nvs_close(handle);

    if (error == ESP_OK) 
    {
        if (servo_id == 1) 
        {
            s_press_force_1 = grams;
        }
        else 
        {
            s_press_force_2 = grams;
        }
    }

    return error;
}

//2026.10.09 新增：获取舵机压力闭环目标克重
esp_err_t servo_get_press_force_setting(int servo_id, float *grams)
{
    if (grams == NULL || servo_id < 1 || servo_id > 2) 
    {
        return ESP_ERR_INVALID_ARG;
    }

    *grams = servo_id == 1 ? s_press_force_1 : s_press_force_2;
    return ESP_OK;
}

//2026.10.09 修改：原实现返回 FSR 实时克重，忽略了 servo_id，
//与 servo_get_press_angle 的语义不一致。
//现改为返回该舵机保存的压力闭环目标克重
float servo_get_press_force(int servo_id)
{
    if (servo_id < 1 || servo_id > 2) 
    {
        return 0.0f;
    }

    return servo_id == 1 ? s_press_force_1 : s_press_force_2;
}

//2026.10.10 新增：保存首次标定得到的最小压力角度
esp_err_t servo_set_press_min_angle(int servo_id, float angle_degrees)
{
    if (servo_id < 1 || servo_id > 2 ||
        angle_degrees < SERVO_PRESS_ANGLE_MIN ||
        angle_degrees > SERVO_PRESS_ANGLE_MAX) {
        return ESP_ERR_INVALID_ARG;
    }
    if (servo_id == 1 && angle_degrees >= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    if (servo_id == 2 && angle_degrees <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }

    float max_abs_angle = servo_get_press_angle(servo_id);
    if (max_abs_angle < 0.0f) {
        max_abs_angle = -max_abs_angle;
    }
    float min_abs_angle = angle_degrees < 0.0f
        ? -angle_degrees
        : angle_degrees;
    if (min_abs_angle > max_abs_angle + 0.01f) {
        return ESP_ERR_INVALID_ARG;
    }

    int32_t stored_angle_x10 = angle_degrees >= 0.0f
        ? (int32_t)(angle_degrees * 10.0f + 0.5f)
        : (int32_t)(angle_degrees * 10.0f - 0.5f);

    nvs_handle_t handle;
    esp_err_t error = nvs_open("servo_cfg", NVS_READWRITE, &handle);
    if (error != ESP_OK) {
        return error;
    }

    error = nvs_set_i32(
        handle,
        servo_press_min_angle_key(servo_id),
        stored_angle_x10);
    if (error == ESP_OK) {
        error = nvs_commit(handle);
    }
    nvs_close(handle);

    if (error == ESP_OK) {
        if (servo_id == 1) {
            s_press_min_angle_1 = angle_degrees;
            s_press_min_angle_1_valid = true;
        } else {
            s_press_min_angle_2 = angle_degrees;
            s_press_min_angle_2_valid = true;
        }
    }

    return error;
}

//2026.10.10 新增：读取首次标定得到的最小压力角度
esp_err_t servo_get_press_min_angle_setting(
    int servo_id,
    float *angle_degrees)
{
    if (angle_degrees == NULL || servo_id < 1 || servo_id > 2) {
        return ESP_ERR_INVALID_ARG;
    }

    bool valid = servo_id == 1
        ? s_press_min_angle_1_valid
        : s_press_min_angle_2_valid;
    if (!valid) {
        return ESP_ERR_NOT_FOUND;
    }

    *angle_degrees = servo_id == 1
        ? s_press_min_angle_1
        : s_press_min_angle_2;
    return ESP_OK;
}

//2026.10.10 新增：判断最小压力角度是否已经标定
bool servo_has_press_min_angle(int servo_id)
{
    if (servo_id == 1) {
        return s_press_min_angle_1_valid;
    }
    if (servo_id == 2) {
        return s_press_min_angle_2_valid;
    }
    return false;
}
