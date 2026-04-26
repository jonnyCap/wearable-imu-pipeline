#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "imu_driver.h"

#if defined(CONFIG_IDF_TARGET_ESP32)
#include "esp_lcd_panel_st7789.h"
#endif

#ifndef GLANCE_DEBUG
#define GLANCE_DEBUG 0
#endif

#define SAMPLE_RATE_HZ                 20
#define SAMPLE_PERIOD_MS               (1000 / SAMPLE_RATE_HZ)
#define WINDOW_DURATION_MS             250
#define WINDOW_SAMPLES                 5
#define STEP_HIGHPASS_CUTOFF_HZ        0.7f
#define STEP_LOWPASS_CUTOFF_HZ         4.0f
#define STEP_REFRACTORY_MS             300
#define GLANCE_STABILITY_ENERGY_ENTER  0.035f
#define GLANCE_STABILITY_ENERGY_EXIT   0.055f
#define GLANCE_ENTER_MEAN_Z_MIN_G      0.70f
#define GLANCE_EXIT_MEAN_Z_MIN_G       0.60f
#define GLANCE_ENTER_MEAN_X_MIN_G      0.18f
#define GLANCE_EXIT_MEAN_X_MIN_G       0.06f
#define GLANCE_ENTER_ABS_X_MAX_G       0.45f
#define GLANCE_EXIT_ABS_X_MAX_G        0.60f
#define GLANCE_ENTER_MAG_MIN_G         0.80f
#define GLANCE_ENTER_MAG_MAX_G         1.20f
#define GLANCE_EXIT_MAG_MIN_G          0.75f
#define GLANCE_EXIT_MAG_MAX_G          1.25f
#define GLANCE_ENTER_HOLD_WINDOWS      2
#define GLANCE_EXIT_HOLD_WINDOWS       2
#define STEP_ACTIVITY_ENTER_G          0.18f
#define STEP_ACTIVITY_EXIT_G           0.12f
#define GLANCE_STATUS_LOG_PERIOD_MS    500
#define DISPLAY_FLUSH_TIMEOUT_TICKS    portMAX_DELAY

#define LCD_H_RES                      240
#define LCD_V_RES                      135
#define LCD_SPI_HOST                   SPI2_HOST
#define LCD_PIN_NUM_CLK                GPIO_NUM_13
#define LCD_PIN_NUM_MOSI               GPIO_NUM_15
#define LCD_PIN_NUM_CS                 GPIO_NUM_5
#define LCD_PIN_NUM_DC                 GPIO_NUM_23
#define LCD_PIN_NUM_RST                GPIO_NUM_18
#define LCD_PCLK_HZ                    (40 * 1000 * 1000)
#define DISPLAY_FONT_MAX_SCALE         6

#if (((SAMPLE_RATE_HZ * WINDOW_DURATION_MS) / 1000) != WINDOW_SAMPLES)
#error "WINDOW_SAMPLES must match SAMPLE_RATE_HZ * 250ms"
#endif

typedef struct {
    bool ready;
    SemaphoreHandle_t flush_sem;
    esp_lcd_panel_io_handle_t io_handle;
    esp_lcd_panel_handle_t panel;
} display_state_t;

typedef struct {
    float x;
    float y;
    float z;
    float mag;
} accel_sample_t;

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float z1;
    float z2;
} biquad_t;

static const char *TAG = "glance_step_detector";

/**
 * @brief Clamp a floating-point value to a closed interval.
 *
 * @param value Input value to constrain.
 * @param minimum Lower bound (inclusive).
 * @param maximum Upper bound (inclusive).
 *
 * @return `value` limited to the `[minimum, maximum]` range.
 */
static inline float float_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) {
        return minimum;
    }
    if (value > maximum) {
        return maximum;
    }
    return value;
}

/**
 * @brief Compute Euclidean acceleration magnitude from axis sample.
 *
 * Uses $\sqrt{x^2 + y^2 + z^2}$ to convert a 3-axis accelerometer sample to
 * a scalar norm in g units.
 *
 * @param sample Accelerometer sample with x/y/z components.
 *
 * @return Magnitude of the acceleration vector in g.
 */
static inline float accel_norm(accel_sample_t sample)
{
    return sqrtf(sample.x * sample.x + sample.y * sample.y + sample.z * sample.z);
}

/**
 * @brief Initialize a second-order low-pass biquad filter.
 *
 * Calculates normalized direct-form coefficients for a Butterworth-like
 * low-pass response (Q ~= 0.7071) at the given cutoff and sample rate, and
 * resets internal delay states.
 *
 * @param[out] filter Filter state/coefficients to initialize.
 * @param sample_rate_hz Sampling rate in Hz.
 * @param cutoff_hz Low-pass cutoff frequency in Hz.
 */
static void biquad_init_lowpass(biquad_t *filter, float sample_rate_hz, float cutoff_hz)
{
    const float q = 0.70710678f;
    const float omega = 2.0f * (float)M_PI * cutoff_hz / sample_rate_hz;
    const float sin_omega = sinf(omega);
    const float cos_omega = cosf(omega);
    const float alpha = sin_omega / (2.0f * q);
    const float a0 = 1.0f + alpha;

    filter->b0 = ((1.0f - cos_omega) * 0.5f) / a0;
    filter->b1 = (1.0f - cos_omega) / a0;
    filter->b2 = ((1.0f - cos_omega) * 0.5f) / a0;
    filter->a1 = (-2.0f * cos_omega) / a0;
    filter->a2 = (1.0f - alpha) / a0;
    filter->z1 = 0.0f;
    filter->z2 = 0.0f;
}

/**
 * @brief Initialize a second-order high-pass biquad filter.
 *
 * Calculates normalized direct-form coefficients for a Butterworth-like
 * high-pass response (Q ~= 0.7071) at the given cutoff and sample rate, and
 * resets internal delay states.
 *
 * @param[out] filter Filter state/coefficients to initialize.
 * @param sample_rate_hz Sampling rate in Hz.
 * @param cutoff_hz High-pass cutoff frequency in Hz.
 */
static void biquad_init_highpass(biquad_t *filter, float sample_rate_hz, float cutoff_hz)
{
    const float q = 0.70710678f;
    const float omega = 2.0f * (float)M_PI * cutoff_hz / sample_rate_hz;
    const float sin_omega = sinf(omega);
    const float cos_omega = cosf(omega);
    const float alpha = sin_omega / (2.0f * q);
    const float a0 = 1.0f + alpha;

    filter->b0 = ((1.0f + cos_omega) * 0.5f) / a0;
    filter->b1 = (-(1.0f + cos_omega)) / a0;
    filter->b2 = ((1.0f + cos_omega) * 0.5f) / a0;
    filter->a1 = (-2.0f * cos_omega) / a0;
    filter->a2 = (1.0f - alpha) / a0;
    filter->z1 = 0.0f;
    filter->z2 = 0.0f;
}

/**
 * @brief Process one sample through a biquad filter section.
 *
 * Implements direct-form II transposed filtering and updates internal delay
 * states for the next sample.
 *
 * @param[in,out] filter Filter coefficients and state memory.
 * @param input Input sample.
 *
 * @return Filtered output sample.
 */
static float biquad_process(biquad_t *filter, float input)
{
    const float output = filter->b0 * input + filter->z1;
    filter->z1 = filter->b1 * input - filter->a1 * output + filter->z2;
    filter->z2 = filter->b2 * input - filter->a2 * output;
    return output;
}

/**
 * @brief Apply cascaded high-pass then low-pass filtering.
 *
 * This constructs a band-pass response suitable for step activity estimation
 * by removing gravity drift first and then suppressing higher-frequency noise.
 *
 * @param[in,out] highpass High-pass filter state.
 * @param[in,out] lowpass Low-pass filter state.
 * @param input Input scalar sample.
 *
 * @return Band-passed sample value.
 */
static float bandpass_process(biquad_t *highpass, biquad_t *lowpass, float input)
{
    return biquad_process(lowpass, biquad_process(highpass, input));
}

/**
 * @brief Convert 8-bit RGB color channels to display-endian RGB565.
 *
 * Packs 24-bit RGB into 16-bit 5:6:5 format and swaps byte order to match the
 * panel transfer format used by this driver path.
 *
 * @param red Red channel (0-255).
 * @param green Green channel (0-255).
 * @param blue Blue channel (0-255).
 *
 * @return Packed RGB565 color value with byte order adjusted for LCD writes.
 */
static uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
    const uint16_t color = (uint16_t)(((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3));
    return (uint16_t)((color >> 8) | (color << 8));
}

/**
 * @brief LCD transfer-complete callback used by asynchronous panel IO.
 *
 * Signals the flush semaphore from ISR context when a queued color transfer
 * finishes so synchronous draw helpers can block until completion.
 *
 * @param panel_io Panel IO handle (unused).
 * @param event_data Transfer event data (unused).
 * @param user_ctx User context containing the flush semaphore handle.
 *
 * @return `true` when a higher-priority task was woken; otherwise `false`.
 */
static bool display_on_color_trans_done(esp_lcd_panel_io_handle_t panel_io,
                                        esp_lcd_panel_io_event_data_t *event_data,
                                        void *user_ctx)
{
    (void)panel_io;
    (void)event_data;

    BaseType_t task_woken = pdFALSE;
    SemaphoreHandle_t flush_sem = (SemaphoreHandle_t)user_ctx;
    xSemaphoreGiveFromISR(flush_sem, &task_woken);
    return task_woken == pdTRUE;
}

/**
 * @brief Draw a bitmap region and block until transfer completion.
 *
 * Clears stale semaphore signals, submits a panel draw command, and waits for
 * the completion callback to release the semaphore before returning.
 *
 * @param[in,out] display Display runtime state.
 * @param x_start Left pixel (inclusive).
 * @param y_start Top pixel (inclusive).
 * @param x_end Right pixel (exclusive).
 * @param y_end Bottom pixel (exclusive).
 * @param color_data Pointer to RGB565 pixel data.
 */
static void display_draw_bitmap_sync(display_state_t *display,
                                     int x_start,
                                     int y_start,
                                     int x_end,
                                     int y_end,
                                     const void *color_data)
{
    if (!display->ready) {
        return;
    }

    while (xSemaphoreTake(display->flush_sem, 0) == pdTRUE) {
    }

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(display->panel, x_start, y_start, x_end, y_end, color_data));
    xSemaphoreTake(display->flush_sem, DISPLAY_FLUSH_TIMEOUT_TICKS);
}

/**
 * @brief Fill the full display with a solid color.
 *
 * Uses a small reusable line-chunk buffer to avoid allocating a full-frame
 * buffer while still minimizing panel transactions.
 *
 * @param[in,out] display Display runtime state.
 * @param color RGB565 color to apply to the whole screen.
 */
static void display_fill_screen(display_state_t *display, uint16_t color)
{
    const int chunk_height = 10;
    uint16_t *buffer = malloc(LCD_H_RES * chunk_height * sizeof(uint16_t));
    if (!buffer) {
        return;
    }

    for (int index = 0; index < LCD_H_RES * chunk_height; ++index) {
        buffer[index] = color;
    }

    for (int y = 0; y < LCD_V_RES; y += chunk_height) {
        const int current_h = (y + chunk_height <= LCD_V_RES) ? chunk_height : (LCD_V_RES - y);
        display_draw_bitmap_sync(display, 0, y, LCD_H_RES, y + current_h, buffer);
    }

    free(buffer);
}

/**
 * @brief Draw a filled rectangle in RGB565 color.
 *
 * Rasterizes one scanline buffer and reuses it for each output row to reduce
 * temporary allocations.
 *
 * @param[in,out] display Display runtime state.
 * @param x Rectangle left coordinate.
 * @param y Rectangle top coordinate.
 * @param width Rectangle width in pixels.
 * @param height Rectangle height in pixels.
 * @param color Fill color in RGB565 format.
 */
static void draw_filled_rect(display_state_t *display, int x, int y, int width, int height, uint16_t color)
{
    if (width <= 0 || height <= 0) {
        return;
    }

    uint16_t row[LCD_H_RES];
    for (int i = 0; i < width && i < LCD_H_RES; ++i) {
        row[i] = color;
    }

    for (int row_index = 0; row_index < height; ++row_index) {
        display_draw_bitmap_sync(display, x, y + row_index, x + width, y + row_index + 1, row);
    }
}

/**
 * @brief Return 5x7 bitmap glyph data for supported characters.
 *
 * Maps a restricted set of uppercase letters and digits to fixed-width glyph
 * bitmaps used by the simple on-device text renderer.
 *
 * @param character ASCII character to map.
 *
 * @return Pointer to 5-byte column-major glyph bitmap; space glyph if unknown.
 */
static const uint8_t *glyph_for_char(char character)
{
    static const uint8_t glyph_space[5] = {0, 0, 0, 0, 0};
    static const uint8_t glyph_0[5] = {0x3E, 0x51, 0x49, 0x45, 0x3E};
    static const uint8_t glyph_1[5] = {0x00, 0x42, 0x7F, 0x40, 0x00};
    static const uint8_t glyph_2[5] = {0x62, 0x51, 0x49, 0x49, 0x46};
    static const uint8_t glyph_3[5] = {0x22, 0x41, 0x49, 0x49, 0x36};
    static const uint8_t glyph_4[5] = {0x18, 0x14, 0x12, 0x7F, 0x10};
    static const uint8_t glyph_5[5] = {0x2F, 0x49, 0x49, 0x49, 0x31};
    static const uint8_t glyph_6[5] = {0x3E, 0x49, 0x49, 0x49, 0x32};
    static const uint8_t glyph_7[5] = {0x01, 0x71, 0x09, 0x05, 0x03};
    static const uint8_t glyph_8[5] = {0x36, 0x49, 0x49, 0x49, 0x36};
    static const uint8_t glyph_9[5] = {0x26, 0x49, 0x49, 0x49, 0x3E};
    static const uint8_t glyph_A[5] = {0x7E, 0x11, 0x11, 0x11, 0x7E};
    static const uint8_t glyph_C[5] = {0x3E, 0x41, 0x41, 0x41, 0x22};
    static const uint8_t glyph_E[5] = {0x7F, 0x49, 0x49, 0x49, 0x41};
    static const uint8_t glyph_G[5] = {0x3E, 0x41, 0x49, 0x49, 0x3A};
    static const uint8_t glyph_L[5] = {0x7F, 0x40, 0x40, 0x40, 0x40};
    static const uint8_t glyph_N[5] = {0x7F, 0x02, 0x0C, 0x10, 0x7F};
    static const uint8_t glyph_O[5] = {0x3E, 0x41, 0x41, 0x41, 0x3E};
    static const uint8_t glyph_P[5] = {0x7F, 0x09, 0x09, 0x09, 0x06};
    static const uint8_t glyph_S[5] = {0x26, 0x49, 0x49, 0x49, 0x32};
    static const uint8_t glyph_T[5] = {0x01, 0x01, 0x7F, 0x01, 0x01};
    static const uint8_t glyph_U[5] = {0x3F, 0x40, 0x40, 0x40, 0x3F};
    static const uint8_t glyph_V[5] = {0x1F, 0x20, 0x40, 0x20, 0x1F};

    switch (character) {
        case '0': return glyph_0;
        case '1': return glyph_1;
        case '2': return glyph_2;
        case '3': return glyph_3;
        case '4': return glyph_4;
        case '5': return glyph_5;
        case '6': return glyph_6;
        case '7': return glyph_7;
        case '8': return glyph_8;
        case '9': return glyph_9;
        case 'A': return glyph_A;
        case 'C': return glyph_C;
        case 'E': return glyph_E;
        case 'G': return glyph_G;
        case 'L': return glyph_L;
        case 'N': return glyph_N;
        case 'O': return glyph_O;
        case 'P': return glyph_P;
        case 'S': return glyph_S;
        case 'T': return glyph_T;
        case 'U': return glyph_U;
        case 'V': return glyph_V;
        default: return glyph_space;
    }
}

/**
 * @brief Draw one scaled bitmap font character.
 *
 * Expands a 5x7 glyph into a caller-selected integer scale, blends foreground
 * and background pixels, and flushes the resulting bitmap to the display.
 *
 * @param[in,out] display Display runtime state.
 * @param x Left pixel position.
 * @param y Top pixel position.
 * @param character ASCII character to render.
 * @param fg_color Foreground text color.
 * @param bg_color Background color.
 * @param scale Integer glyph scaling factor (clamped to supported range).
 */
static void display_draw_char(display_state_t *display,
                              int x,
                              int y,
                              char character,
                              uint16_t fg_color,
                              uint16_t bg_color,
                              int scale)
{
    if (scale < 1) {
        scale = 1;
    }
    if (scale > DISPLAY_FONT_MAX_SCALE) {
        scale = DISPLAY_FONT_MAX_SCALE;
    }

    const uint8_t *glyph = glyph_for_char(character);
    const int width = 6 * scale;
    const int height = 8 * scale;
    uint16_t *buffer = malloc(width * height * sizeof(uint16_t));
    if (!buffer) {
        return;
    }

    for (int row = 0; row < height; ++row) {
        const int source_row = row / scale;
        for (int column = 0; column < width; ++column) {
            const int source_column = column / scale;
            bool pixel_on = false;
            if (source_column < 5 && source_row < 7) {
                pixel_on = ((glyph[source_column] >> source_row) & 0x01) != 0;
            }
            buffer[row * width + column] = pixel_on ? fg_color : bg_color;
        }
    }

    display_draw_bitmap_sync(display, x, y, x + width, y + height, buffer);
    free(buffer);
}

/**
 * @brief Draw a text string using the internal fixed bitmap font.
 *
 * Renders characters left-to-right at fixed cell spacing, supporting simple
 * whitespace handling and caller-defined foreground/background colors.
 *
 * @param[in,out] display Display runtime state.
 * @param x Left pixel start position.
 * @param y Top pixel baseline position.
 * @param text Null-terminated string to render.
 * @param fg_color Foreground text color.
 * @param bg_color Background color.
 * @param scale Integer glyph scaling factor.
 */
static void display_draw_text(display_state_t *display,
                              int x,
                              int y,
                              const char *text,
                              uint16_t fg_color,
                              uint16_t bg_color,
                              int scale)
{
    int cursor_x = x;
    while (*text != '\0') {
        if (*text == ' ') {
            cursor_x += 6 * scale;
        } else {
            display_draw_char(display, cursor_x, y, *text, fg_color, bg_color, scale);
            cursor_x += 6 * scale;
        }
        ++text;
    }
}

/**
 * @brief Initialize LCD bus, panel IO, and panel state.
 *
 * Configures SPI transport, panel IO callbacks, ST7789 panel device state, and
 * orientation parameters, then clears the screen. On unsupported targets,
 * gracefully reports `ESP_ERR_NOT_SUPPORTED`.
 *
 * @param[out] display Display state structure to initialize.
 *
 * @return
 * - ESP_OK when the display is ready.
 * - ESP_ERR_NO_MEM if semaphore allocation fails.
 * - ESP_ERR_NOT_SUPPORTED on non-ESP32 target path.
 * - ESP_ERR_* code from bus/panel initialization failures.
 */
static esp_err_t display_init(display_state_t *display)
{
    memset(display, 0, sizeof(*display));
    display->flush_sem = xSemaphoreCreateBinary();
    if (display->flush_sem == NULL) {
        return ESP_ERR_NO_MEM;
    }

#if defined(CONFIG_IDF_TARGET_ESP32)
    spi_bus_config_t bus_config = {
        .sclk_io_num = LCD_PIN_NUM_CLK,
        .mosi_io_num = LCD_PIN_NUM_MOSI,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * 40 * sizeof(uint16_t),
    };

    esp_err_t err = spi_bus_initialize(LCD_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGW(TAG, "LCD SPI bus init failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_NUM_DC,
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = LCD_PCLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config, &display->io_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LCD panel IO init failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_io_callbacks_t cbs = {
        .on_color_trans_done = display_on_color_trans_done,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(display->io_handle, &cbs, display->flush_sem));

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    err = esp_lcd_new_panel_st7789(display->io_handle, &panel_config, &display->panel);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LCD panel init failed: %s", esp_err_to_name(err));
        return err;
    }

    ESP_ERROR_CHECK(esp_lcd_panel_reset(display->panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(display->panel));
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(display->panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(display->panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(display->panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(display->panel, true, false));
    ESP_ERROR_CHECK(esp_lcd_panel_set_gap(display->panel, 40, 52));

    display->ready = true;
    display_fill_screen(display, rgb565(0, 0, 0));
    return ESP_OK;
#else
    if (display->flush_sem != NULL) {
        vSemaphoreDelete(display->flush_sem);
        display->flush_sem = NULL;
    }
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

/**
 * @brief Render the inactive/hidden screen state.
 *
 * Clears the panel to black when display output is available.
 *
 * @param[in,out] display Display runtime state.
 */
static void display_show_hidden(display_state_t *display)
{
    if (!display->ready) {
        return;
    }

    display_fill_screen(display, rgb565(0, 0, 0));
}

/**
 * @brief Render the numeric step count in the center content area.
 *
 * Formats the 32-bit step count, computes centered placement, paints a backing
 * strip, and draws the enlarged number text.
 *
 * @param[in,out] display Display runtime state.
 * @param step_count Current accumulated step count.
 */
static void display_draw_step_count(display_state_t *display, uint32_t step_count)
{
    if (!display->ready) {
        return;
    }

    char step_text[32];
    snprintf(step_text, sizeof(step_text), "%" PRIu32, step_count);

    const int scale = 4;
    const int text_width = (int)strlen(step_text) * 6 * scale;
    const int text_x = (LCD_H_RES - text_width) / 2;

    draw_filled_rect(display, 0, 42, LCD_H_RES, 56, rgb565(2, 8, 6));
    display_draw_text(display, text_x, 48, step_text, rgb565(255, 255, 255), rgb565(2, 8, 6), scale);
}

/**
 * @brief Render the active glance UI screen with heading and step count.
 *
 * Draws the panel background, a heading banner, and the current counter value
 * when the wrist pose indicates glance mode is active.
 *
 * @param[in,out] display Display runtime state.
 * @param step_count Current accumulated step count.
 */
static void display_show_active(display_state_t *display, uint32_t step_count)
{
    if (!display->ready) {
        return;
    }

    display_fill_screen(display, rgb565(2, 8, 6));
    draw_filled_rect(display, 12, 12, 216, 24, rgb565(0, 116, 84));
    display_draw_text(display, 72, 18, "STEPS", rgb565(255, 255, 255), rgb565(0, 116, 84), 2);
    display_draw_step_count(display, step_count);
}

/**
 * @brief Render the inactive display state.
 *
 * Wrapper that currently maps inactive mode to a hidden/blanked screen for
 * future flexibility if additional inactive visuals are introduced.
 *
 * @param[in,out] display Display runtime state.
 */
static void display_show_inactive(display_state_t *display)
{
    display_show_hidden(display);
}

/**
 * @brief Emit verbose per-window debug sample metrics when enabled.
 *
 * Logging is compiled in only when `GLANCE_DEBUG` is non-zero; otherwise
 * parameters are explicitly marked unused.
 *
 * @param x Mean/sample X acceleration in g.
 * @param y Mean/sample Y acceleration in g.
 * @param z Mean/sample Z acceleration in g.
 * @param magnitude Mean acceleration magnitude in g.
 * @param motion_level Scalar motion metric derived from variance.
 * @param activity_peak_to_peak Band-passed step signal peak-to-peak amplitude.
 * @param glance_active Current glance-state flag.
 * @param step_count Current step count.
 */
static void log_sample_debug(float x,
                             float y,
                             float z,
                             float magnitude,
                             float motion_level,
                             float activity_peak_to_peak,
                             bool glance_active,
                             uint32_t step_count)
{
#if GLANCE_DEBUG
    ESP_LOGI(TAG,
             "accel=(%.3f, %.3f, %.3f) mag=%.3f motion=%.3f p2p=%.3f glance=%d steps=%" PRIu32,
             x, y, z, magnitude, motion_level, activity_peak_to_peak, glance_active, step_count);
#else
    (void)x;
    (void)y;
    (void)z;
    (void)magnitude;
    (void)motion_level;
    (void)activity_peak_to_peak;
    (void)glance_active;
    (void)step_count;
#endif
}

/**
 * @brief Periodically log glance-state classifier internals.
 *
 * Reports pose means, magnitude, stability indicator, and energy metrics to
 * help tune threshold values and diagnose false positives/negatives.
 *
 * @param mean_x Mean X acceleration over current window.
 * @param mean_y Mean Y acceleration over current window.
 * @param mean_z Mean Z acceleration over current window.
 * @param stability_energy Summed per-axis variance metric.
 * @param mean_magnitude Mean acceleration magnitude over window.
 * @param stable_window Whether current window passed stability gate.
 * @param glance_active Current glance-state flag.
 */
static void log_glance_status(float mean_x,
                              float mean_y,
                              float mean_z,
                              float stability_energy,
                              float mean_magnitude,
                              bool stable_window,
                              bool glance_active)
{
    const float stability_headroom = GLANCE_STABILITY_ENERGY_ENTER - stability_energy;

    if (!glance_active) {
        ESP_LOGI(TAG,
                 "glance_check mean=(%.2f, %.2f, %.2f) mag=%.2fg stable=%d energy=%.4f headroom=%.4f",
                 mean_x,
                 mean_y,
                 mean_z,
                 mean_magnitude,
                 stable_window,
                 stability_energy,
                 stability_headroom);
    } else {
        ESP_LOGI(TAG,
                 "glance_active mean=(%.2f, %.2f, %.2f) mag=%.2fg stable=%d energy=%.4f",
                 mean_x,
                 mean_y,
                 mean_z,
                 mean_magnitude,
                 stable_window,
                 stability_energy);
    }
}

/**
 * @brief Log buffered samples used to validate a detected step.
 *
 * Emits one line per window entry with raw axis values, magnitude, and the
 * corresponding filtered step signal.
 *
 * @param window Buffered accelerometer samples for the current detection window.
 * @param filtered_window Buffered band-passed step signal values.
 */
static void log_step_detection_buffer(const accel_sample_t window[WINDOW_SAMPLES],
                                      const float filtered_window[WINDOW_SAMPLES])
{
    ESP_LOGI(TAG, "Step window dump (%d samples)", WINDOW_SAMPLES);
    for (size_t i = 0; i < WINDOW_SAMPLES; ++i) {
        ESP_LOGI(TAG,
                 "  [%u] x=%.3f y=%.3f z=%.3f mag=%.3f filt=%.3f",
                 (unsigned)i,
                 window[i].x,
                 window[i].y,
                 window[i].z,
                 window[i].mag,
                 filtered_window[i]);
    }
}

/**
 * @brief Main application loop for glance-triggered step visualization.
 *
 * Initializes IMU and display resources, samples accelerometer data at the
 * configured rate, computes windowed orientation/stability features for glance
 * state transitions with hysteresis, detects steps from a band-passed
 * magnitude signal with refractory handling, updates the on-device display, and
 * emits periodic diagnostic logs.
 */
void app_main(void)
{
    ESP_ERROR_CHECK(imu_init());

    display_state_t display = {0};
    if (display_init(&display) != ESP_OK) {
        ESP_LOGW(TAG, "LCD output unavailable; running with serial logs only");
    }

    bool glance_active = false;
    bool step_activity_high = false;
    bool last_glance_display_state = false;
    uint8_t glance_enter_count = 0;
    uint8_t glance_exit_count = 0;
    uint32_t step_count = 0;
    accel_sample_t window[WINDOW_SAMPLES] = {0};
    float step_signal_window[WINDOW_SAMPLES] = {0};
    size_t window_count = 0;
    float last_window_stability_energy = 0.0f;
    float last_window_activity = 0.0f;
    float last_mean_magnitude = 0.0f;
    bool last_window_stable = false;
    float last_mean_x = 0.0f;
    float last_mean_y = 0.0f;
    float last_mean_z = 0.0f;
    biquad_t step_highpass = {0};
    biquad_t step_lowpass = {0};
    TickType_t last_status_log_tick = 0;
    TickType_t last_step_tick = 0;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t last_rendered_step_count = UINT32_MAX;

    biquad_init_highpass(&step_highpass, SAMPLE_RATE_HZ, STEP_HIGHPASS_CUTOFF_HZ);
    biquad_init_lowpass(&step_lowpass, SAMPLE_RATE_HZ, STEP_LOWPASS_CUTOFF_HZ);

    display_show_inactive(&display);

    while (true) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(SAMPLE_PERIOD_MS));

        float x = 0.0f;
        float y = 0.0f;
        float z = 0.0f;
        esp_err_t read_result = imu_read_accel(&x, &y, &z);
        if (read_result != ESP_OK) {
            ESP_LOGW(TAG, "IMU read failed: %s", esp_err_to_name(read_result));
            continue;
        }

        const float sample_magnitude = accel_norm((accel_sample_t){
            .x = x,
            .y = y,
            .z = z,
        });

        window[window_count] = (accel_sample_t){
            .x = x,
            .y = y,
            .z = z,
            .mag = sample_magnitude,
        };
        step_signal_window[window_count] = bandpass_process(&step_highpass,
                                                             &step_lowpass,
                                                             sample_magnitude);
        window_count += 1;

        if (window_count < WINDOW_SAMPLES) {
            continue;
        }

        float sum_x = 0.0f;
        float sum_y = 0.0f;
        float sum_z = 0.0f;
        float sum_magnitude = 0.0f;
        float sum_square = 0.0f;
        float min_filtered = INFINITY;
        float max_filtered = -INFINITY;
        for (size_t i = 0; i < WINDOW_SAMPLES; ++i) {
            const accel_sample_t sample = window[i];
            const float magnitude = sample.mag;
            const float filtered_step_signal = step_signal_window[i];

            sum_x += sample.x;
            sum_y += sample.y;
            sum_z += sample.z;
            sum_magnitude += magnitude;
            sum_square += magnitude * magnitude;
            if (filtered_step_signal < min_filtered) {
                min_filtered = filtered_step_signal;
            }
            if (filtered_step_signal > max_filtered) {
                max_filtered = filtered_step_signal;
            }
        }

        const float window_inv = 1.0f / (float)WINDOW_SAMPLES;
        const float mean_x = sum_x * window_inv;
        const float mean_y = sum_y * window_inv;
        const float mean_z = sum_z * window_inv;
        const float mean_magnitude = sum_magnitude * window_inv;
        const float variance = float_clamp((sum_square * window_inv) - (mean_magnitude * mean_magnitude), 0.0f, 100.0f);

        // Stateless stability gate: low per-axis variance means the wrist is holding a pose.
        float variance_x_sum = 0.0f;
        float variance_y_sum = 0.0f;
        float variance_z_sum = 0.0f;
        for (size_t i = 0; i < WINDOW_SAMPLES; ++i) {
            const float dx = window[i].x - mean_x;
            const float dy = window[i].y - mean_y;
            const float dz = window[i].z - mean_z;
            variance_x_sum += dx * dx;
            variance_y_sum += dy * dy;
            variance_z_sum += dz * dz;
        }

        const float variance_x = variance_x_sum * window_inv;
        const float variance_y = variance_y_sum * window_inv;
        const float variance_z = variance_z_sum * window_inv;
        const float stability_energy = variance_x + variance_y + variance_z;
        const float filtered_peak_to_peak = max_filtered - min_filtered;

        last_mean_x = mean_x;
        last_mean_y = mean_y;
        last_mean_z = mean_z;
        last_mean_magnitude = mean_magnitude;
        last_window_stability_energy = stability_energy;
        last_window_activity = filtered_peak_to_peak;
        last_window_stable = glance_active ?
            (stability_energy < GLANCE_STABILITY_ENERGY_EXIT) :
            (stability_energy < GLANCE_STABILITY_ENERGY_ENTER);

        // Board mounting maps wrist-right tilt primarily to X, while Y captures front/back.
        const bool pose_enter =
            (mean_z > GLANCE_ENTER_MEAN_Z_MIN_G) &&
            (mean_x > GLANCE_ENTER_MEAN_X_MIN_G) &&
            (fabsf(mean_y) < GLANCE_ENTER_ABS_X_MAX_G) &&
            (mean_magnitude > GLANCE_ENTER_MAG_MIN_G) &&
            (mean_magnitude < GLANCE_ENTER_MAG_MAX_G);

        const bool pose_exit =
            (mean_z < GLANCE_EXIT_MEAN_Z_MIN_G) ||
            (mean_x < GLANCE_EXIT_MEAN_X_MIN_G) ||
            (fabsf(mean_y) > GLANCE_EXIT_ABS_X_MAX_G) ||
            (mean_magnitude < GLANCE_EXIT_MAG_MIN_G) ||
            (mean_magnitude > GLANCE_EXIT_MAG_MAX_G);

        const bool glance_enter_candidate = last_window_stable && pose_enter;
        const bool glance_exit_candidate = (!last_window_stable) || pose_exit;

        // Window-level hysteresis to avoid flicker around orientation boundaries.
        if (!glance_active) {
            if (glance_enter_candidate) {
                if (glance_enter_count < GLANCE_ENTER_HOLD_WINDOWS) {
                    glance_enter_count += 1;
                }
            } else {
                glance_enter_count = 0;
            }
            glance_exit_count = 0;

            if (glance_enter_count >= GLANCE_ENTER_HOLD_WINDOWS) {
                glance_active = true;
                glance_enter_count = 0;
            }
        } else {
            if (glance_exit_candidate) {
                if (glance_exit_count < GLANCE_EXIT_HOLD_WINDOWS) {
                    glance_exit_count += 1;
                }
            } else {
                glance_exit_count = 0;
            }
            glance_enter_count = 0;

            if (glance_exit_count >= GLANCE_EXIT_HOLD_WINDOWS) {
                glance_active = false;
                glance_exit_count = 0;
            }
        }

        if (!step_activity_high && filtered_peak_to_peak >= STEP_ACTIVITY_ENTER_G) {
            step_activity_high = true;

            const TickType_t now = xTaskGetTickCount();
            const bool refractory_met = (now - last_step_tick) >= pdMS_TO_TICKS(STEP_REFRACTORY_MS);
            if (refractory_met) {
                step_count += 1;
                last_step_tick = now;
                log_step_detection_buffer(window, step_signal_window);
                if (display.ready && glance_active) {
                    display_draw_step_count(&display, step_count);
                    last_rendered_step_count = step_count;
                }
                ESP_LOGI(TAG, "Step detected, count=%" PRIu32, step_count);
            }
        } else if (step_activity_high && filtered_peak_to_peak <= STEP_ACTIVITY_EXIT_G) {
            step_activity_high = false;
        }

        if (glance_active != last_glance_display_state) {
            if (glance_active) {
                display_show_active(&display, step_count);
                last_rendered_step_count = step_count;
            } else {
                display_show_inactive(&display);
                last_rendered_step_count = UINT32_MAX;
            }
            last_glance_display_state = glance_active;
        }

        if (glance_active && display.ready && step_count != last_rendered_step_count) {
            display_draw_step_count(&display, step_count);
            last_rendered_step_count = step_count;
        }

        const TickType_t now = xTaskGetTickCount();
        if ((now - last_status_log_tick) >= pdMS_TO_TICKS(GLANCE_STATUS_LOG_PERIOD_MS)) {
            log_glance_status(last_mean_x,
                              last_mean_y,
                              last_mean_z,
                              last_window_stability_energy,
                              last_mean_magnitude,
                              last_window_stable,
                              glance_active);
            last_status_log_tick = now;
        }

        log_sample_debug(x,
                         y,
                         z,
                         mean_magnitude,
                         sqrtf(variance),
                         last_window_activity,
                         glance_active,
                         step_count);

        window_count = 0;
    }
}