#include "system_commands.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#include "driver/uart.h"
#include "esp_log.h"
#include "esp_private/esp_clk.h"
#include "esp_sleep.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

#define COMMAND_UART       UART_NUM_0
#define APP_VERSION        "FWRI-SJ-A1-V001"
#define HARDWARE_VERSION   "HWRI-SJ-A1-V001"
#define DEFAULT_SN         "0000000000000000"
#define SN_MAX_LENGTH      32

static const char *TAG = "system_commands";

static void send_text(const char *text)
{
    uart_write_bytes(COMMAND_UART, text, strlen(text));
}

static void send_run_over(void)
{
    send_text("run over\r\n");
}

static bool sn_is_valid(const char *sn)
{
    size_t length = strlen(sn);
    if (length == 0 || length > SN_MAX_LENGTH) {
        return false;
    }

    for (size_t i = 0; i < length; ++i) {
        unsigned char character = (unsigned char)sn[i];
        if (!isalnum(character) &&
            character != '-' &&
            character != '_') {
            return false;
        }
    }
    return true;
}

static esp_err_t read_sn(char *sn, size_t capacity)
{
    nvs_handle_t handle;
    esp_err_t error =
        nvs_open("device", NVS_READONLY, &handle);
    if (error != ESP_OK) {
        return error;
    }

    size_t length = capacity;
    error = nvs_get_str(handle, "sn", sn, &length);
    nvs_close(handle);

    if (error == ESP_ERR_NVS_NOT_FOUND) {
        if (strlen(DEFAULT_SN) + 1 > capacity) {
            return ESP_ERR_INVALID_SIZE;
        }
        strcpy(sn, DEFAULT_SN);
        return ESP_OK;
    }
    return error;
}

static esp_err_t write_sn(const char *sn)
{
    nvs_handle_t handle;
    esp_err_t error =
        nvs_open("device", NVS_READWRITE, &handle);
    if (error != ESP_OK) {
        return error;
    }

    error = nvs_set_str(handle, "sn", sn);
    if (error == ESP_OK) {
        error = nvs_commit(handle);
    }
    nvs_close(handle);
    return error;
}

esp_err_t system_commands_init(void)
{
    return nvs_flash_init();
}

bool system_command_handle(const char *line)
{
    if (strcmp(line, "AT+LSYSOSC?") == 0) {
        send_text(
            esp_clk_xtal_freq() > 0
                ? "+LSYSOSC:OK\r\n"
                : "+LSYSOSC:ERROR\r\n");
        send_run_over();
        return true;
    }

    if (strcmp(line, "AT+LRTCOSC?") == 0) {
#if defined(CONFIG_RTC_CLK_SRC_EXT_CRYS) || \
    defined(CONFIG_RTC_CLK_SRC_EXT_OSC)
        send_text("+LRTCOSC:OK\r\n");
#else
        send_text("+LRTCOSC:ERROR\r\n");
#endif
        send_run_over();
        return true;
    }

    if (strcmp(line, "AT+LSYSFRY?") == 0) {
        char response[48];
        int frequency = esp_clk_cpu_freq();
        if (frequency > 0) {
            snprintf(
                response,
                sizeof(response),
                "+LSYSFRY:%dHz\r\nOK\r\n",
                frequency);
            send_text(response);
        } else {
            send_text("+LGETSYSFRY:ERROR\r\n");
        }
        send_run_over();
        return true;
    }

    if (strcmp(line, "AT+LAPPVER?") == 0) {
        send_text("+LAPPVER:" APP_VERSION "\r\nOK\r\n");
        send_run_over();
        return true;
    }

    if (strcmp(line, "AT+LHARDVER?") == 0) {
        send_text("+LHARDVER:" HARDWARE_VERSION "\r\nOK\r\n");
        send_run_over();
        return true;
    }

    if (strcmp(line, "AT+LGETSN?") == 0) {
        char sn[SN_MAX_LENGTH + 1];
        if (read_sn(sn, sizeof(sn)) == ESP_OK) {
            char response[64];
            snprintf(
                response,
                sizeof(response),
                "+LGETSN:%s\r\nOK\r\n",
                sn);
            send_text(response);
        } else {
            send_text("+LGETSN:ERROR\r\n");
        }
        send_run_over();
        return true;
    }

    const char set_sn_prefix[] = "AT+LSETSN=";
    if (strncmp(
            line,
            set_sn_prefix,
            sizeof(set_sn_prefix) - 1) == 0) {
        const char *sn = line + sizeof(set_sn_prefix) - 1;
        if (!sn_is_valid(sn) || write_sn(sn) != ESP_OK) {
            send_text("+LSETSN:ERROR\r\n");
        } else {
            char response[64];
            snprintf(
                response,
                sizeof(response),
                "OK\r\n+LSETSN:%s\r\n",
                sn);
            send_text(response);
        }
        send_run_over();
        return true;
    }

    if (strcmp(line, "AT+LSLEEP") == 0) {
        esp_err_t error =
            uart_set_wakeup_threshold(COMMAND_UART, 3);
        if (error == ESP_OK) {
            error =
                esp_sleep_enable_uart_wakeup(COMMAND_UART);
        }
        if (error != ESP_OK) {
            send_text("+LSLEEP:ERROR\r\n");
            send_run_over();
            return true;
        }

        send_text("OK\r\n+LSLEEP:OK\r\n");
        send_run_over();
        uart_wait_tx_done(COMMAND_UART, pdMS_TO_TICKS(100));
        ESP_LOGI(TAG, "Entering light sleep");
        error = esp_light_sleep_start();
        if (error != ESP_OK) {
            ESP_LOGE(
                TAG,
                "Light sleep failed: %s",
                esp_err_to_name(error));
        } else {
            ESP_LOGI(TAG, "Woke from light sleep");
        }
        return true;
    }

    return false;
}
