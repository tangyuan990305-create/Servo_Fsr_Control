#ifndef DRV_OLED_H
#define DRV_OLED_H

#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define DRV_OLED_WIDTH   128
#define DRV_OLED_HEIGHT  32

/**
 * @brief Initialize the onboard SSD1306 display and its I2C bus.
 *
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t drv_oled_init(void);

/**
 * @brief Clear the local display framebuffer.
 *
 * Call drv_oled_refresh() to send the cleared framebuffer to the display.
 */
void drv_oled_clear(void);

/**
 * @brief Draw supported uppercase text into the local framebuffer.
 *
 * The current font supports the uppercase letters and symbols used by the
 * application, digits, space, plus and minus.
 *
 * @param x Horizontal start position in pixels.
 * @param y Vertical start position in pixels.
 * @param text Null-terminated text to draw.
 * @param scale Integer pixel scale; values below 1 are treated as 1.
 */
void drv_oled_draw_text(int x, int y, const char *text, uint8_t scale);

/**
 * @brief Transfer the local framebuffer to the OLED.
 *
 * @return ESP_OK on success, otherwise an ESP-IDF error code.
 */
esp_err_t drv_oled_refresh(void);

#ifdef __cplusplus
}
#endif

#endif
