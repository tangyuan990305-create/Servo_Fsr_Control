#include "drv_oled.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "driver/i2c.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define OLED_I2C_PORT       I2C_NUM_0
#define OLED_SDA_GPIO       GPIO_NUM_21
#define OLED_SCL_GPIO       GPIO_NUM_22
#define OLED_I2C_ADDRESS    0x3C
#define OLED_I2C_FREQUENCY  400000

#define OLED_PAGE_COUNT   (DRV_OLED_HEIGHT / 8)
#define OLED_BUFFER_SIZE  (DRV_OLED_WIDTH * OLED_PAGE_COUNT)

static const char *TAG = "drv_oled";
static uint8_t s_framebuffer[OLED_BUFFER_SIZE];
static bool s_initialized;

static esp_err_t oled_write_command(uint8_t command)
{
    const uint8_t packet[] = {
        0x00, /* Command control byte. */
        command,
    };

    return i2c_master_write_to_device(
        OLED_I2C_PORT,
        OLED_I2C_ADDRESS,
        packet,
        sizeof(packet),
        pdMS_TO_TICKS(100));
}

static void oled_set_pixel(int x, int y)
{
    if (x < 0 || x >= DRV_OLED_WIDTH ||
        y < 0 || y >= DRV_OLED_HEIGHT) {
        return;
    }

    size_t index = (size_t)x + ((size_t)y / 8U) * DRV_OLED_WIDTH;
    s_framebuffer[index] |= (uint8_t)(1U << ((uint32_t)y % 8U));
}

static const uint8_t *oled_get_glyph(char character)
{
    static const uint8_t glyph_h[] = {0x7F, 0x08, 0x08, 0x08, 0x7F};
    static const uint8_t glyph_a[] = {0x7E, 0x09, 0x09, 0x09, 0x7E};
    static const uint8_t glyph_d[] = {0x7F, 0x41, 0x41, 0x22, 0x1C};
    static const uint8_t glyph_e[] = {0x7F, 0x49, 0x49, 0x49, 0x41};
    static const uint8_t glyph_g[] = {0x3E, 0x41, 0x49, 0x49, 0x3A};
    static const uint8_t glyph_l[] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const uint8_t glyph_m[] = {0x7F, 0x02, 0x0C, 0x02, 0x7F};
    static const uint8_t glyph_n[] = {0x7F, 0x02, 0x0C, 0x10, 0x7F};
    static const uint8_t glyph_o[] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
    static const uint8_t glyph_plus[] = {0x08, 0x08, 0x3E, 0x08, 0x08};
    static const uint8_t glyph_minus[] = {0x08, 0x08, 0x08, 0x08, 0x08};
    static const uint8_t glyph_digits[][5] = {
        {0x3E, 0x51, 0x49, 0x45, 0x3E},
        {0x00, 0x42, 0x7F, 0x40, 0x00},
        {0x42, 0x61, 0x51, 0x49, 0x46},
        {0x21, 0x41, 0x45, 0x4B, 0x31},
        {0x18, 0x14, 0x12, 0x7F, 0x10},
        {0x27, 0x45, 0x45, 0x45, 0x39},
        {0x3C, 0x4A, 0x49, 0x49, 0x30},
        {0x01, 0x71, 0x09, 0x05, 0x03},
        {0x36, 0x49, 0x49, 0x49, 0x36},
        {0x06, 0x49, 0x49, 0x29, 0x1E},
    };

    switch (character) {
    case 'A':
        return glyph_a;
    case 'G':
        return glyph_g;
    case 'D':
        return glyph_d;
    case 'H':
        return glyph_h;
    case 'E':
        return glyph_e;
    case 'L':
        return glyph_l;
    case 'M':
        return glyph_m;
    case 'N':
        return glyph_n;
    case 'O':
        return glyph_o;
    case '+':
        return glyph_plus;
    case '-':
        return glyph_minus;
    case ' ':
        return NULL;
    default:
        if (character >= '0' && character <= '9') {
            return glyph_digits[character - '0'];
        }
        return NULL;
    }
}

static void oled_draw_character(
    int x,
    int y,
    char character,
    uint8_t scale)
{
    const uint8_t *glyph = oled_get_glyph(character);
    if (glyph == NULL) {
        return;
    }

    for (int column = 0; column < 5; ++column) {
        for (int row = 0; row < 7; ++row) {
            if ((glyph[column] & (1U << row)) == 0) {
                continue;
            }

            for (uint8_t dx = 0; dx < scale; ++dx) {
                for (uint8_t dy = 0; dy < scale; ++dy) {
                    oled_set_pixel(
                        x + column * scale + dx,
                        y + row * scale + dy);
                }
            }
        }
    }
}

esp_err_t drv_oled_init(void)
{
    if (s_initialized) {
        return ESP_OK;
    }

    const i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = OLED_SDA_GPIO,
        .scl_io_num = OLED_SCL_GPIO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = OLED_I2C_FREQUENCY,
        .clk_flags = 0,
    };

    ESP_RETURN_ON_ERROR(
        i2c_param_config(OLED_I2C_PORT, &config),
        TAG,
        "I2C configuration failed");

    ESP_RETURN_ON_ERROR(
        i2c_driver_install(OLED_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0),
        TAG,
        "I2C driver installation failed");

    const uint8_t init_commands[] = {
        0xAE,             /* Display off. */
        0xD5, 0x80,       /* Display clock. */
        0xA8, 0x1F,       /* Multiplex ratio: 32 rows. */
        0xD3, 0x00,       /* Display offset. */
        0x40,             /* Display start line. */
        0x8D, 0x14,       /* Enable charge pump. */
        0x20, 0x00,       /* Horizontal addressing mode. */
        0xA1,             /* Segment remap. */
        0xC8,             /* COM scan direction. */
        0xDA, 0x02,       /* COM pin configuration. */
        0x81, 0x8F,       /* Contrast. */
        0xD9, 0xF1,       /* Pre-charge period. */
        0xDB, 0x40,       /* VCOMH deselect level. */
        0xA4,             /* Display framebuffer contents. */
        0xA6,             /* Normal display mode. */
        0xAF,             /* Display on. */
    };

    for (size_t i = 0; i < sizeof(init_commands); ++i) {
        ESP_RETURN_ON_ERROR(
            oled_write_command(init_commands[i]),
            TAG,
            "OLED initialization command failed");
    }

    s_initialized = true;
    drv_oled_clear();
    return ESP_OK;
}

void drv_oled_clear(void)
{
    memset(s_framebuffer, 0, sizeof(s_framebuffer));
}

void drv_oled_draw_text(
    int x,
    int y,
    const char *text,
    uint8_t scale)
{
    if (text == NULL) {
        return;
    }

    if (scale == 0) {
        scale = 1;
    }

    while (*text != '\0') {
        oled_draw_character(x, y, *text, scale);
        x += 6 * scale;
        ++text;
    }
}

esp_err_t drv_oled_refresh(void)
{
    if (!s_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    ESP_RETURN_ON_ERROR(oled_write_command(0x21), TAG, "Set column failed");
    ESP_RETURN_ON_ERROR(oled_write_command(0), TAG, "Set column start failed");
    ESP_RETURN_ON_ERROR(
        oled_write_command(DRV_OLED_WIDTH - 1),
        TAG,
        "Set column end failed");

    ESP_RETURN_ON_ERROR(oled_write_command(0x22), TAG, "Set page failed");
    ESP_RETURN_ON_ERROR(oled_write_command(0), TAG, "Set page start failed");
    ESP_RETURN_ON_ERROR(
        oled_write_command(OLED_PAGE_COUNT - 1),
        TAG,
        "Set page end failed");

    for (int page = 0; page < OLED_PAGE_COUNT; ++page) {
        uint8_t packet[DRV_OLED_WIDTH + 1];

        packet[0] = 0x40; /* Display data control byte. */
        memcpy(
            &packet[1],
            &s_framebuffer[page * DRV_OLED_WIDTH],
            DRV_OLED_WIDTH);

        ESP_RETURN_ON_ERROR(
            i2c_master_write_to_device(
                OLED_I2C_PORT,
                OLED_I2C_ADDRESS,
                packet,
                sizeof(packet),
                pdMS_TO_TICKS(100)),
            TAG,
            "OLED framebuffer transfer failed");
    }

    return ESP_OK;
}
