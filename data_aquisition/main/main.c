/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"
#include "imu_driver.h"
#include "secrets.h"  // Include WiFi credentials and other secrets

// Configuration Parameters - update these according to your local network and server setup
#ifndef WIFI_SSID 
#define WIFI_SSID      "YOUR_WIFI_SSID"
#endif
#ifndef WIFI_PASS
#define WIFI_PASS      "YOUR_WIFI_PASSWORD"
#endif

#define SERVER_IP      "192.168.0.51"
#define SERVER_PORT    8080

// Button configuration for M5Stack
#define BUTTON_GPIO    GPIO_NUM_39  // Button A on M5Stack
#define BUTTON_DEBOUNCE_MS 50

// Task specifications based on Task.md recommendations
#define SAMPLE_RATE_HZ     20
#define SAMPLE_INTERVAL_MS (1000 / SAMPLE_RATE_HZ)
#define BATCH_SIZE         10  // Number of samples to send in a single TCP packet
#define IMU_STREAMING_TASK_STACK_SIZE 8192
#define PAYLOAD_BATCH_CAPACITY 2048
#define READING_BUFFER_CAPACITY 160
#define BUTTON_POLL_MS 10
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_CONNECT_TIMEOUT_MS 10000
#define SOCKET_RETRY_DELAY_MS 2000

static const char *TAG = "imu_client";
static TimerHandle_t wifi_reconnect_timer;
static EventGroupHandle_t wifi_event_group;
static char current_session_uuid[37] = {0};

static TickType_t last_button_press_tick = 0;
static int last_button_level = 1;

static bool send_all(int sock, const char *data, size_t len) {
    size_t total_sent = 0;
    while (total_sent < len) {
        int written = send(sock, data + total_sent, len - total_sent, 0);
        if (written <= 0) {
            return false;
        }
        total_sent += (size_t)written;
    }
    return true;
}

static void log_streaming_stack_watermark(void) {
    UBaseType_t min_free_words = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGI(TAG, "imu_streaming minimum free stack: %u bytes", (unsigned int)(min_free_words * sizeof(StackType_t)));
}

static void generate_session_uuid(char *uuid_buf, size_t buf_len) {
    uint8_t raw[16];
    esp_fill_random(raw, sizeof(raw));

    // Set UUID version (4) and variant (RFC 4122).
    raw[6] = (raw[6] & 0x0F) | 0x40;
    raw[8] = (raw[8] & 0x3F) | 0x80;

    snprintf(uuid_buf, buf_len,
             "%02x%02x%02x%02x-%02x%02x-%02x%02x-%02x%02x-%02x%02x%02x%02x%02x%02x",
             raw[0], raw[1], raw[2], raw[3],
             raw[4], raw[5],
             raw[6], raw[7],
             raw[8], raw[9],
             raw[10], raw[11], raw[12], raw[13], raw[14], raw[15]);
}

// Read accelerometer data from MPU6050 IMU
void get_accelerometer_data(float *x, float *y, float *z) {
    esp_err_t ret = imu_read_accel(x, y, z);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer: %s", esp_err_to_name(ret));
        // Return default values on error
        *x = 0.0f;
        *y = 0.0f;
        *z = 0.0f;
    }
}

static bool button_pressed_event(void) {
    int current_level = gpio_get_level(BUTTON_GPIO);
    bool pressed = false;

    // Falling edge detection (active-low button)
    if (last_button_level == 1 && current_level == 0) {
        TickType_t now = xTaskGetTickCount();
        if ((now - last_button_press_tick) >= pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS)) {
            // Debounce check: button must remain low after short delay
            vTaskDelay(pdMS_TO_TICKS(BUTTON_DEBOUNCE_MS));
            if (gpio_get_level(BUTTON_GPIO) == 0) {
                pressed = true;
                last_button_press_tick = xTaskGetTickCount();
            }
        }
    }

    last_button_level = current_level;
    return pressed;
}

static void wifi_reconnect_timer_callback(TimerHandle_t timer) {
    (void)timer;
    esp_err_t err = esp_wifi_connect();
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "WiFi reconnect failed: %s", esp_err_to_name(err));
    }
}

void button_init(void) {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << BUTTON_GPIO),
        .mode = GPIO_MODE_INPUT,
        // GPIO39 is input-only; use the board's external pull-up instead of an internal one.
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,  // Trigger on button press (falling edge)
    };
    
    ESP_ERROR_CHECK(gpio_config(&io_conf));
    last_button_level = gpio_get_level(BUTTON_GPIO);
    last_button_press_tick = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "Button initialized on GPIO %d", BUTTON_GPIO);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
        ESP_LOGI(TAG, "WiFi disconnected, retrying in 5 seconds...");
        if (wifi_reconnect_timer != NULL) {
            xTimerReset(wifi_reconnect_timer, 0);
        } else {
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init_sta(void) {
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(wifi_event_group != NULL ? ESP_OK : ESP_FAIL);

    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
        },
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
    wifi_reconnect_timer = xTimerCreate("wifi_reconnect", pdMS_TO_TICKS(5000), pdFALSE, NULL, wifi_reconnect_timer_callback);
    esp_wifi_start();
    
    ESP_LOGI(TAG, "WiFi initialization complete.");
}

void imu_streaming_task(void *pvParameters) {
    (void)pvParameters;
    int addr_family = 0;
    int ip_protocol = 0;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SERVER_PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    char *payload_batch = malloc(PAYLOAD_BATCH_CAPACITY);
    char *current_reading = malloc(READING_BUFFER_CAPACITY);
    if (payload_batch == NULL || current_reading == NULL) {
        ESP_LOGE(TAG, "Failed to allocate streaming buffers");
        free(payload_batch);
        free(current_reading);
        vTaskDelete(NULL);
    }

    while (1) {
        // Wait for one explicit button press to start a recording session.
        ESP_LOGI(TAG, "Waiting for button press to start recording...");
        while (!button_pressed_event()) {
            vTaskDelay(pdMS_TO_TICKS(BUTTON_POLL_MS));
        }

        bool recording_active = true;

        generate_session_uuid(current_session_uuid, sizeof(current_session_uuid));
        ESP_LOGI(TAG, "New recording session UUID: %s", current_session_uuid);
        
        if ((xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) == 0) {
            ESP_LOGI(TAG, "Waiting for WiFi IP before connecting to server...");
        }

        while (recording_active && (xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) == 0) {
            EventBits_t bits = xEventGroupWaitBits(
                wifi_event_group,
                WIFI_CONNECTED_BIT,
                pdFALSE,
                pdFALSE,
                pdMS_TO_TICKS(WIFI_CONNECT_TIMEOUT_MS)
            );
            if ((bits & WIFI_CONNECTED_BIT) == 0) {
                ESP_LOGW(TAG, "Still waiting for WiFi connection...");
            }

            // Allow cancelling if user pressed button again before connect.
            if (button_pressed_event()) {
                recording_active = false;
            }
        }

        if (!recording_active) {
            ESP_LOGI(TAG, "Recording canceled before connection; waiting for next button press...");
            continue;
        }

        ESP_LOGI(TAG, "Recording state changed, attempting connection...");

        int sock = -1;
        while (recording_active) {
            if ((xEventGroupGetBits(wifi_event_group) & WIFI_CONNECTED_BIT) == 0) {
                ESP_LOGW(TAG, "WiFi lost while preparing connection; waiting to reconnect...");
                vTaskDelay(pdMS_TO_TICKS(500));

                if (button_pressed_event()) {
                    recording_active = false;
                }
                continue;
            }

            sock = socket(addr_family, SOCK_STREAM, ip_protocol);
            if (sock < 0) {
                ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
                vTaskDelay(pdMS_TO_TICKS(SOCKET_RETRY_DELAY_MS));
                continue;
            }

            ESP_LOGI(TAG, "Socket created, connecting to %s:%d", SERVER_IP, SERVER_PORT);
            int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err == 0) {
                break;
            }

            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            sock = -1;
            vTaskDelay(pdMS_TO_TICKS(SOCKET_RETRY_DELAY_MS));

            if (button_pressed_event()) {
                recording_active = false;
            }
        }

        if (!recording_active || sock < 0) {
            if (sock != -1) {
                close(sock);
            }
            ESP_LOGI(TAG, "Streaming session ended before socket connect.");
            continue;
        }

        ESP_LOGI(TAG, "Successfully connected");

        payload_batch[0] = '\0';
        size_t payload_len = 0;
        int batch_count = 0;
        bool send_failed = false;

        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_INTERVAL_MS);

        // Send data until user presses the button again to stop.
        while (recording_active) {
            float x, y, z;
            get_accelerometer_data(&x, &y, &z);

            // Construct newline-delimited JSON payload
            int reading_len = snprintf(current_reading, READING_BUFFER_CAPACITY,
                                       "{\"uuid\": \"%s\", \"x\": %.4f, \"y\": %.4f, \"z\": %.4f}\n",
                                       current_session_uuid, x, y, z);
            if (reading_len < 0 || reading_len >= READING_BUFFER_CAPACITY) {
                ESP_LOGE(TAG, "JSON reading formatting failed/truncated");
                send_failed = true;
                break;
            }

            // Flush if the next reading would overflow the batch buffer.
            if (batch_count > 0 && payload_len + (size_t)reading_len >= PAYLOAD_BATCH_CAPACITY) {
                if (!send_all(sock, payload_batch, payload_len)) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    send_failed = true;
                    break;
                }
                payload_len = 0;
                payload_batch[0] = '\0';
                batch_count = 0;
            }

            memcpy(payload_batch + payload_len, current_reading, (size_t)reading_len);
            payload_len += (size_t)reading_len;
            payload_batch[payload_len] = '\0';
            batch_count++;

            // If we reached the batch limit, send the batch buffer to the server
            if (batch_count >= BATCH_SIZE) {
                if (!send_all(sock, payload_batch, payload_len)) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    send_failed = true;
                    break;
                }
                
                // Reset batch buffer
                payload_len = 0;
                payload_batch[0] = '\0';
                batch_count = 0;
            }

            if (button_pressed_event()) {
                recording_active = false;
            }

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }

        // Send any remaining data in batch buffer
        if (!send_failed && batch_count > 0 && payload_len > 0) {
            if (!send_all(sock, payload_batch, payload_len)) {
                ESP_LOGE(TAG, "Error occurred during final batch send: errno %d", errno);
                send_failed = true;
            }
        }

        // Recording stopped, send the "final" flag
        if (!send_failed) {
            ESP_LOGI(TAG, "Transmitting completion status");
            char final_payload[100];
            int final_len = snprintf(final_payload, sizeof(final_payload), "{\"uuid\": \"%s\", \"final\": true}\n", current_session_uuid);
            if (final_len > 0 && final_len < (int)sizeof(final_payload)) {
                if (!send_all(sock, final_payload, (size_t)final_len)) {
                    ESP_LOGE(TAG, "Error occurred during final marker send: errno %d", errno);
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and stopping transmission");
            shutdown(sock, 0);
            close(sock);
        }

        log_streaming_stack_watermark();
        ESP_LOGI(TAG, "Streaming complete. Waiting for next button press...");
    }
}

void app_main(void)
{
    // Initialize NVS (needed by WiFi internally)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Initialize IMU (MPU6050)
    ESP_LOGI(TAG, "Initializing IMU...");
    ESP_ERROR_CHECK(imu_init());

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
    
    // Initialize button for recording control
    button_init();

    // Start IMU Data streaming task with extra stack headroom for networking and JSON formatting.
    BaseType_t task_created = xTaskCreate(imu_streaming_task, "imu_streaming", IMU_STREAMING_TASK_STACK_SIZE, NULL, 5, NULL);
    ESP_ERROR_CHECK(task_created == pdPASS ? ESP_OK : ESP_FAIL);
}
