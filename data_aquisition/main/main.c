/*
 * SPDX-FileCopyrightText: 2010-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "lwip/sockets.h"
#include "driver/gpio.h"
#include "imu_driver.h"

// Configuration Parameters - update these according to your local network and server setup
#define WIFI_SSID      "YOUR_WIFI_SSID"
#define WIFI_PASS      "YOUR_WIFI_PASSWORD"
#define SERVER_IP      "192.168.1.100"  // Your computer's IP address
#define SERVER_PORT    8080
#define SESSION_UUID   "session-0001"

// Button configuration for M5Stack
#define BUTTON_GPIO    GPIO_NUM_39  // Button A on M5Stack
#define BUTTON_DEBOUNCE_MS 50

// Task specifications based on Task.md recommendations
#define SAMPLE_RATE_HZ     20
#define SAMPLE_INTERVAL_MS (1000 / SAMPLE_RATE_HZ)
#define BATCH_SIZE         10  // Number of samples to send in a single TCP packet

static const char *TAG = "imu_client";
static TimerHandle_t wifi_reconnect_timer;

// Recording state: 0 = idle, 1 = recording, 2 = stop requested
static volatile int recording_state = 0;
static uint32_t last_button_press = 0;

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

static void button_isr_handler(void* arg) {
    uint32_t current_time = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS;
    
    // Debounce: ignore if pressed within BUTTON_DEBOUNCE_MS
    if (current_time - last_button_press < BUTTON_DEBOUNCE_MS) {
        return;
    }
    last_button_press = current_time;
    
    // Toggle recording state: idle -> recording -> stop -> idle
    if (recording_state == 0) {
        recording_state = 1;  // Start recording
    } else if (recording_state == 1) {
        recording_state = 2;  // Stop recording
    }
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
    ESP_ERROR_CHECK(gpio_install_isr_service(0));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, button_isr_handler, NULL));
    
    ESP_LOGI(TAG, "Button initialized on GPIO %d", BUTTON_GPIO);
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi disconnected, retrying in 5 seconds...");
        if (wifi_reconnect_timer != NULL) {
            xTimerReset(wifi_reconnect_timer, 0);
        } else {
            esp_wifi_connect();
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
    }
}

void wifi_init_sta(void) {
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
    int addr_family = 0;
    int ip_protocol = 0;

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(SERVER_PORT);
    addr_family = AF_INET;
    ip_protocol = IPPROTO_IP;

    while (1) {
        // Wait for button press to start recording
        while (recording_state == 0) {
            vTaskDelay(100 / portTICK_PERIOD_MS);
        }
        
        ESP_LOGI(TAG, "Recording state changed, attempting connection...");

        // Simple connection retry loop
        int sock = socket(addr_family, SOCK_STREAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            recording_state = 0;  // Reset state on error
            continue;
        }

        ESP_LOGI(TAG, "Socket created, connecting to %s:%d", SERVER_IP, SERVER_PORT);
        int err = connect(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err != 0) {
            ESP_LOGE(TAG, "Socket unable to connect: errno %d", errno);
            close(sock);
            recording_state = 0;  // Reset state on error
            vTaskDelay(2000 / portTICK_PERIOD_MS);
            continue;
        }

        ESP_LOGI(TAG, "Successfully connected");

        char payload_batch[2048] = {0}; // Big buffer to hold multiple JSON lines
        int batch_count = 0;

        TickType_t xLastWakeTime = xTaskGetTickCount();
        const TickType_t xFrequency = pdMS_TO_TICKS(SAMPLE_INTERVAL_MS);

        // Send data while recording (recording_state == 1)
        while (recording_state == 1) {
            float x, y, z;
            get_accelerometer_data(&x, &y, &z);

            // Construct newline-delimited JSON payload
            char current_reading[150];
            snprintf(current_reading, sizeof(current_reading), 
                     "{\"uuid\": \"%s\", \"x\": %.4f, \"y\": %.4f, \"z\": %.4f}\n", 
                     SESSION_UUID, x, y, z);
            
            // Append it to our batch buffer
            strcat(payload_batch, current_reading);
            batch_count++;

            // If we reached the batch limit, send the batch buffer to the server
            if (batch_count >= BATCH_SIZE) {
                int written = send(sock, payload_batch, strlen(payload_batch), 0);
                if (written < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
                
                // Reset batch buffer
                memset(payload_batch, 0, sizeof(payload_batch));
                batch_count = 0;
            }

            vTaskDelayUntil(&xLastWakeTime, xFrequency);
        }

        // Send any remaining data in batch buffer
        if (batch_count > 0) {
            int written = send(sock, payload_batch, strlen(payload_batch), 0);
            if (written < 0) {
                ESP_LOGE(TAG, "Error occurred during final batch send: errno %d", errno);
            }
        }

        // Recording stopped, send the "final" flag
        ESP_LOGI(TAG, "Transmitting completion status");
        char final_payload[100];
        snprintf(final_payload, sizeof(final_payload), "{\"uuid\": \"%s\", \"final\": true}\n", SESSION_UUID);
        send(sock, final_payload, strlen(final_payload), 0);

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and stopping transmission");
            shutdown(sock, 0);
            close(sock);
        }

        // Reset state and wait for next button press
        recording_state = 0;
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

    // Start IMU Data streaming task
    xTaskCreate(imu_streaming_task, "imu_streaming", 4096, NULL, 5, NULL);
}
