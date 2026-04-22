#include <inttypes.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
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
#define GLANCE_WINDOW_SAMPLES          5
#define STARTUP_CALIBRATION_SAMPLES    (SAMPLE_RATE_HZ * 2)
#define STEP_REFRactory_MS             350
#define GLANCE_ENTER_DISTANCE_G        0.22f
#define GLANCE_EXIT_DISTANCE_G         0.14f
#define GLANCE_HOLD_MS                 1500
#define STEP_MIN_THRESHOLD             0.12f
#define STEP_THRESHOLD_MULTIPLIER      2.2f
#define STEP_NOISE_ALPHA               0.025f
#define GRAVITY_TOLERANCE_G            0.35f

#define LCD_H_RES                      320
#define LCD_V_RES                      240
#define LCD_SPI_HOST                   SPI2_HOST
#define LCD_PIN_NUM_CLK                GPIO_NUM_18
#define LCD_PIN_NUM_MOSI               GPIO_NUM_23
#define LCD_PIN_NUM_CS                 GPIO_NUM_14
#define LCD_PIN_NUM_DC                 GPIO_NUM_27
#define LCD_PIN_NUM_RST                GPIO_NUM_33
#define LCD_PIN_NUM_BCKL               GPIO_NUM_32
#define LCD_PCLK_HZ                    (40 * 1000 * 1000)

typedef struct {
    float b0;
    float b1;
    float b2;
    float a1;
    float a2;
    float z1;
    float z2;
} biquad_t;

typedef struct {
    bool ready;
    esp_lcd_panel_handle_t panel;
} display_state_t;

typedef struct {
    float x;
    float y;
    float z;
} accel_sample_t;

static const char *TAG = "glance_step_detector";

static inline float float_max(float lhs, float rhs)
{
    return lhs > rhs ? lhs : rhs;
}

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

static float biquad_process(biquad_t *filter, float input)
{
    const float output = filter->b0 * input + filter->z1;
    filter->z1 = filter->b1 * input - filter->a1 * output + filter->z2;
    filter->z2 = filter->b2 * input - filter->a2 * output;
    return output;
}

static float bandpass_process(biquad_t *highpass, biquad_t *lowpass, float input)
{
    return biquad_process(lowpass, biquad_process(highpass, input));
}

static uint16_t rgb565(uint8_t red, uint8_t green, uint8_t blue)
{
    return (uint16_t)(((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3));
}

static void display_fill_screen(esp_lcd_panel_handle_t panel, uint16_t color)
{
    uint16_t row[LCD_H_RES];
    for (size_t index = 0; index < LCD_H_RES; ++index) {
        row[index] = color;
    }

    for (int y = 0; y < LCD_V_RES; ++y) {
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, 0, y, LCD_H_RES, y + 1, row));
    }
}

static void draw_filled_rect(esp_lcd_panel_handle_t panel, int x, int y, int width, int height, uint16_t color)
{
    if (width <= 0 || height <= 0) {
        return;
    }

    uint16_t row[LCD_H_RES];
    for (int i = 0; i < width && i < LCD_H_RES; ++i) {
        row[i] = color;
    }

    for (int row_index = 0; row_index < height; ++row_index) {
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, x, y + row_index, x + width, y + row_index + 1, row));
    }
}

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

static void display_draw_char(esp_lcd_panel_handle_t panel, int x, int y, char character, uint16_t fg_color, uint16_t bg_color, int scale)
{
    const uint8_t *glyph = glyph_for_char(character);
    const int width = 6 * scale;
    const int height = 8 * scale;
    uint16_t buffer[6 * 8 * 4 * 4];

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

    ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(panel, x, y, x + width, y + height, buffer));
}

static void display_draw_text(esp_lcd_panel_handle_t panel, int x, int y, const char *text, uint16_t fg_color, uint16_t bg_color, int scale)
{
    int cursor_x = x;
    while (*text != '\0') {
        if (*text == ' ') {
            cursor_x += 6 * scale;
        } else {
            display_draw_char(panel, cursor_x, y, *text, fg_color, bg_color, scale);
            cursor_x += 6 * scale;
        }
        ++text;
    }
}

static esp_err_t display_init(display_state_t *display)
{
    memset(display, 0, sizeof(*display));

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

    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_PIN_NUM_DC,
        .cs_gpio_num = LCD_PIN_NUM_CS,
        .pclk_hz = LCD_PCLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };

    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_SPI_HOST, &io_config, &io_handle);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "LCD panel IO init failed: %s", esp_err_to_name(err));
        return err;
    }

    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_PIN_NUM_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_BGR,
        .bits_per_pixel = 16,
    };

    err = esp_lcd_new_panel_st7789(io_handle, &panel_config, &display->panel);
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

    gpio_config_t backlight_config = {
        .pin_bit_mask = 1ULL << LCD_PIN_NUM_BCKL,
        .mode = GPIO_MODE_OUTPUT,
    };
    ESP_ERROR_CHECK(gpio_config(&backlight_config));
    ESP_ERROR_CHECK(gpio_set_level(LCD_PIN_NUM_BCKL, 1));

    display->ready = true;
    display_fill_screen(display->panel, rgb565(0, 0, 0));
    return ESP_OK;
#else
    return ESP_ERR_NOT_SUPPORTED;
#endif
}

static void display_show_calibrating(display_state_t *display)
{
    if (!display->ready) {
        return;
    }

    display_fill_screen(display->panel, rgb565(0, 0, 0));
    draw_filled_rect(display->panel, 18, 16, 284, 28, rgb565(16, 48, 96));
    display_draw_text(display->panel, 42, 24, "CALIBRATING", rgb565(255, 255, 255), rgb565(16, 48, 96), 2);
}

static void display_show_hidden(display_state_t *display)
{
    if (!display->ready) {
        return;
    }

    display_fill_screen(display->panel, rgb565(0, 0, 0));
}

static void display_show_active(display_state_t *display, uint32_t step_count)
{
    if (!display->ready) {
        return;
    }

    char step_text[32];
    snprintf(step_text, sizeof(step_text), "%" PRIu32, step_count);

    display_fill_screen(display->panel, rgb565(4, 12, 8));
    draw_filled_rect(display->panel, 16, 14, 288, 34, rgb565(0, 128, 96));
    display_draw_text(display->panel, 44, 22, "GLANCE ACTIVE", rgb565(255, 255, 255), rgb565(0, 128, 96), 2);
    display_draw_text(display->panel, 52, 100, "STEPS", rgb565(130, 255, 190), rgb565(4, 12, 8), 3);
    display_draw_text(display->panel, 112, 140, step_text, rgb565(255, 255, 255), rgb565(4, 12, 8), 4);
}

static void display_show_inactive(display_state_t *display)
{
    display_show_hidden(display);
}

static void log_sample_debug(float x, float y, float z, float magnitude, float filtered, float threshold, bool glance_active, uint32_t step_count)
{
#if GLANCE_DEBUG
    ESP_LOGI(TAG,
             "accel=(%.3f, %.3f, %.3f) mag=%.3f filtered=%.3f threshold=%.3f glance=%d steps=%" PRIu32,
             x, y, z, magnitude, filtered, threshold, glance_active, step_count);
#else
    (void)x;
    (void)y;
    (void)z;
    (void)magnitude;
    (void)filtered;
    (void)threshold;
    (void)glance_active;
    (void)step_count;
#endif
}

void app_main(void)
{
    display_state_t display = {0};
    if (display_init(&display) != ESP_OK) {
        ESP_LOGW(TAG, "LCD output unavailable; running with serial logs only");
    }

    ESP_ERROR_CHECK(imu_init());

    display_show_calibrating(&display);

    biquad_t highpass;
    biquad_t lowpass;
    biquad_init_highpass(&highpass, SAMPLE_RATE_HZ, 0.5f);
    biquad_init_lowpass(&lowpass, SAMPLE_RATE_HZ, 6.0f);

    accel_sample_t baseline = {0};
    float baseline_magnitude = 1.0f;
    float baseline_magnitude_sum = 0.0f;

    accel_sample_t glance_window[GLANCE_WINDOW_SAMPLES] = {0};
    int glance_window_index = 0;
    int glance_window_count = 0;

    bool calibrated = false;
    bool glance_active = false;
    bool last_glance_display_state = false;
    uint32_t step_count = 0;
    float filtered_noise_floor = 0.05f;
    float previous_filtered = 0.0f;
    float previous_previous_filtered = 0.0f;
    TickType_t glance_hold_until = 0;
    TickType_t last_step_tick = 0;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t calibration_samples = 0;

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

        const float magnitude = sqrtf(x * x + y * y + z * z);

        if (!calibrated) {
            baseline.x += x;
            baseline.y += y;
            baseline.z += z;
            baseline_magnitude_sum += magnitude;
            calibration_samples += 1;

            if (calibration_samples >= STARTUP_CALIBRATION_SAMPLES) {
                const float sample_count = (float)calibration_samples;
                baseline.x /= sample_count;
                baseline.y /= sample_count;
                baseline.z /= sample_count;
                baseline_magnitude = baseline_magnitude_sum / sample_count;
                calibrated = true;
                display_show_inactive(&display);
                ESP_LOGI(TAG, "Calibration complete: baseline=(%.3f, %.3f, %.3f) magnitude=%.3f",
                         baseline.x, baseline.y, baseline.z, baseline_magnitude);
            }
            continue;
        }

        const float centered_magnitude = magnitude - baseline_magnitude;
        const float filtered = bandpass_process(&highpass, &lowpass, centered_magnitude);

        filtered_noise_floor = (1.0f - STEP_NOISE_ALPHA) * filtered_noise_floor + STEP_NOISE_ALPHA * fabsf(filtered);
        const float step_threshold = float_max(STEP_MIN_THRESHOLD, filtered_noise_floor * STEP_THRESHOLD_MULTIPLIER + 0.02f);

        if (previous_previous_filtered < previous_filtered && previous_filtered >= filtered) {
            const TickType_t now = xTaskGetTickCount();
            const bool threshold_met = previous_filtered > step_threshold;
            const bool refractory_met = (now - last_step_tick) >= pdMS_TO_TICKS(STEP_REFRactory_MS);

            if (threshold_met && refractory_met) {
                step_count += 1;
                last_step_tick = now;
                if (display.ready && glance_active) {
                    display_show_active(&display, step_count);
                }
                ESP_LOGI(TAG, "Step detected, count=%" PRIu32, step_count);
            }
        }

        previous_previous_filtered = previous_filtered;
        previous_filtered = filtered;

        glance_window[glance_window_index].x = x;
        glance_window[glance_window_index].y = y;
        glance_window[glance_window_index].z = z;
        glance_window_index = (glance_window_index + 1) % GLANCE_WINDOW_SAMPLES;
        if (glance_window_count < GLANCE_WINDOW_SAMPLES) {
            glance_window_count += 1;
        }

        if (glance_window_count == GLANCE_WINDOW_SAMPLES) {
            accel_sample_t mean = {0};
            for (int index = 0; index < GLANCE_WINDOW_SAMPLES; ++index) {
                mean.x += glance_window[index].x;
                mean.y += glance_window[index].y;
                mean.z += glance_window[index].z;
            }

            const float sample_count = (float)GLANCE_WINDOW_SAMPLES;
            mean.x /= sample_count;
            mean.y /= sample_count;
            mean.z /= sample_count;

            const float distance = sqrtf(
                (mean.x - baseline.x) * (mean.x - baseline.x) +
                (mean.y - baseline.y) * (mean.y - baseline.y) +
                (mean.z - baseline.z) * (mean.z - baseline.z));
            const float magnitude_error = fabsf(magnitude - baseline_magnitude);

            if (!glance_active) {
                if (distance > GLANCE_ENTER_DISTANCE_G && magnitude_error < GRAVITY_TOLERANCE_G) {
                    glance_active = true;
                    glance_hold_until = xTaskGetTickCount() + pdMS_TO_TICKS(GLANCE_HOLD_MS);
                }
            } else {
                if (distance > GLANCE_ENTER_DISTANCE_G) {
                    glance_hold_until = xTaskGetTickCount() + pdMS_TO_TICKS(GLANCE_HOLD_MS);
                } else if (distance < GLANCE_EXIT_DISTANCE_G && xTaskGetTickCount() > glance_hold_until) {
                    glance_active = false;
                }
            }

            if (glance_active != last_glance_display_state) {
                if (glance_active) {
                    display_show_active(&display, step_count);
                } else {
                    display_show_inactive(&display);
                }
                last_glance_display_state = glance_active;
            } else if (glance_active && display.ready) {
                display_show_active(&display, step_count);
            }

            log_sample_debug(x, y, z, magnitude, filtered, step_threshold, glance_active, step_count);
        }
    }
}