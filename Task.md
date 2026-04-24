## Goals
This exercise will be the first step into the domain of human activity recognition, aiming for gesture recognition (the glance at the wearable's screen) and step detection (in a walking activity).

First, you will implement a simple windowing approach to cut incoming sensor data into smaller chunks and then analyze the accelerometer data to detect the gesture/posture of reading the device's screen: lifting the arm and holding the arm in front of the chest to read the display.

Then, using recorded data samples from different activities (e.g., walking, standing still, stepping quickly), you will determine the filter parameters to detect walking accurately. Once these parameters are determined, you will apply them in real-time to filter incoming IMU sensor data on the wearable device and recognize walking activity effectively.

## Task Description
**(GO FOR THE FULL POINTS VERSION ONLY)**

### Glance on Screen Detection (3 points)
For the wearable device, implement a windowing approach (Tutorial 7, no overlap necessary here) that buffers 250 ms of accelerometer data, sampled, e.g., at 20 Hz, and then analyzes the signals from the x, y, and z axes to detect the gesture/posture of reading the device's screen. Keep the approach simple (KISS design principle), so aggregate the buffered data, e.g., using the mean, and use simple thresholds (ideally with error margin and hysteresis). Show a simple text or illustration on the screen when the gesture/posture is detected (and clear it afterward) to demonstrate the working feature in the video submission.

### Step Detection & Counting (6 points) 
In this task, you will develop a step-counting algorithm that combines the windowing concept (Task 1) with filters to clean the raw signals obtained from the sensor.

**Data Acquisition**
First, implement software that allows you to stream the accelerometer measurements to your computer, and then record and save them into a *.csv file. We recommend a sampling rate of at least 20 Hz. With this tool, record at least 60 seconds of data with the device worn at either wrist, repeating a cycle of 15s walking and 15s standing twice (so walk, stand, walk, and stand).

**Offline Analysis**
Use the provided Jupyter Notebook to analyze the data (based on Tutorial 6). You can use the FFT (fast Fourier transform) or Welch's method (average of multiple FFTs on data segments) to obtain the frequency spectrum or periodogram, respectively. Based on your insights, decide on a suitable aggregation technique as well as filters and filter parameters (e.g., cutoff frequency) to preprocess the raw signals. With the target application of step counting in mind, use the provided Python functions to test your concept and configuration.

**Wearable Implementation & Step Detection**
Finally, you will transfer your system concept to the wearable device. Use the provided Python script to calculate the filter coefficients for implementing the filters (Tutorial 6), enabling real-time signal processing on the wearable. Keep in mind that you can peek into the black box and test your filter implementation by streaming the signals to your computer at any time. Eventually, implement the rest of the concept by implementing aggregation, windowing (Tutorial 7, Task 1), and step feature detection. Base your implementation on realistic and feasible assumptions on the step activity (e.g., step frequency and interval) to determine, e.g., window size, overlap, and threshold levels. Count the number of steps the user is performing.

### Display Results
Show the count only when the reading gesture (Task 1) is detected.

### Reflection (1 point)
a) First, write a brief reflection about your experience with the tasks. Cover your gained knowledge, insights, and identified topics where you need further clarification or faced challenges.

b) Afterward, write down your thoughts, in particular on the following aspects:
* Which values did you obtain for cutoff frequencies?
* How would these cutoff frequencies have to be adjusted to detect different activities, e.g., running?

## Example Windowing
```C++
/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
// Include the I2C master driver header
#include "driver/i2c_master.h"

static const char *TAG = "i2c_mpu6886_example_terminal";

// Global handles for the I2C bus and the device
i2c_master_dev_handle_t dev_handle = NULL;
i2c_master_bus_handle_t bus_handle = NULL;

#define WINDOW_SIZE 5

// -------------------------------------------------------------------------
// I2C Master Configuration
// -------------------------------------------------------------------------
#define I2C_MASTER_SCL_IO           22                         /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           21                         /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              0                          /*!< I2C master i2c port number */
#define I2C_MASTER_FREQ_HZ          400000                     /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                          /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// -------------------------------------------------------------------------
// MPU6886 Register Definitions
// -------------------------------------------------------------------------
#define MPU6886_SENSOR_ADDR                     0x68        /*!< Slave address of the MPU6866 sensor */
#define MPU6886_WHO_AM_I_REG_ADDR               0x75        /*!< Register addresses of the "who am I" register */
#define MPU6886_SMPLRT_DIV_REG_ADDR             0x19
#define MPU6886_CONFIG_REG_ADDR                 0x1A
#define MPU6886_ACCEL_CONFIG_REG_ADDR           0x1C
#define MPU6886_ACCEL_CONFIG_2_REG_ADDR         0x1D
#define MPU6886_FIFO_EN_REG_ADDR                0x23
#define MPU6886_INT_PIN_CFG_REG_ADDR            0x37
#define MPU6886_INT_ENABLE_REG_ADDR             0x38
#define MPU6886_ACCEL_XOUT_REG_ADDR             0x3B
#define MPU6886_USER_CRTL_REG_ADDR              0x6A
#define MPU6886_PWR_MGMT_1_REG_ADDR             0x6B
#define MPU6886_PWR_MGMT_2_REG_ADDR             0x6C

// -------------------------------------------------------------------------
// I2C Communication Primitives
// -------------------------------------------------------------------------

/**
 * @brief Reads multiple bytes from a specific register.
 * * First transmits the register address, then receives the data.
 */
static esp_err_t mpu6886_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    esp_err_t ret;
    extern i2c_master_dev_handle_t dev_handle; // declare external device handle

    // Write register address first
    ret = i2c_master_transmit(dev_handle, &reg_addr, 1, 10*I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    if (ret != ESP_OK) return ret;

    // Read data from device
    ret = i2c_master_receive(dev_handle, data, len, 10*I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
    return ret;
}

/**
 * @brief Writes a single byte to a specific register.
 */
static esp_err_t mpu6886_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    esp_err_t ret;
    uint8_t write_buf[2] = {reg_addr, data};
    extern i2c_master_dev_handle_t dev_handle; // declare external device handle

    ret = i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), 10*I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);

    return ret;
}

// -------------------------------------------------------------------------
// Hardware Initialization
// -------------------------------------------------------------------------

/**
 * @brief Configures the MPU6886 sensor via I2C commands.
 * * Includes power management, sample rate, and accelerometer configuration.
 */
static void init_mpu6886(void)
{
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1_REG_ADDR, (0x01 << 7)));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_PWR_MGMT_1_REG_ADDR, (0x01 << 0)));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_ACCEL_CONFIG_REG_ADDR, 0x18));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_CONFIG_REG_ADDR, 0x01));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_SMPLRT_DIV_REG_ADDR, 0x05));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_ENABLE_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_ACCEL_CONFIG_2_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_USER_CRTL_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_FIFO_EN_REG_ADDR, 0x00));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_PIN_CFG_REG_ADDR, 0x22));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_ERROR_CHECK(mpu6886_register_write_byte(MPU6886_INT_ENABLE_REG_ADDR, 0x01));
    vTaskDelay(10 / portTICK_PERIOD_MS);
    ESP_LOGI(TAG, "MPU6866 intialized successfully");
    vTaskDelay(10 / portTICK_PERIOD_MS);
}

/**
 * @brief Initializes the ESP32 I2C Master bus and adds the MPU6886 device.
 */
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6886_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

// -------------------------------------------------------------------------
// Data Retrieval
// -------------------------------------------------------------------------

/**
 * @brief Reads raw ADC values for X, Y, and Z axes.
 */
static void getAccelAdc(int16_t* ax, int16_t* ay, int16_t* az)
{
    uint8_t buf[6];
    mpu6886_register_read(MPU6886_ACCEL_XOUT_REG_ADDR, buf, 6);

    *ax = ((int16_t)buf[0] << 8) | buf[1];
    *ay = ((int16_t)buf[2] << 8) | buf[3];
    *az = ((int16_t)buf[4] << 8) | buf[5];
}

/**
 * @brief Converts raw ADC values to gravitational force (G).
 */
static void getAccelData(float* ax, float* ay, float* az)
{
    int16_t accX = 0;
    int16_t accY = 0;
    int16_t accZ = 0;
    getAccelAdc(&accX, &accY, &accZ);
    float aRes = 16.0 / 32768.0;

    *ax = (float)accX * aRes;
    *ay = (float)accY * aRes;
    *az = (float)accZ * aRes;
}

// -------------------------------------------------------------------------
// Main Application
// -------------------------------------------------------------------------
void app_main(void)
{
    // Initialize I2C Peripheral
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    // Initialize MPU6886 Sensor
    init_mpu6886();

    float gx, gy, gz;
    float x_buffer[WINDOW_SIZE];
    float y_buffer[WINDOW_SIZE];
    float z_buffer[WINDOW_SIZE];

    int index = 0;

    while (1)
    {
       getAccelData(&gx, &gy, &gz);

        // store the new sensor data at the current buffer position
        x_buffer[index] = gx;
        y_buffer[index] = gy;
        z_buffer[index] = gz;
        index++;

        if (index == WINDOW_SIZE) // window is full
        {
            // compute the mean
            float mean_x = 0.0f;
            float mean_y = 0.0f;
            float mean_z = 0.0f;

            for (int i = 0; i < WINDOW_SIZE; i++)
            {
                mean_x += x_buffer[i];
                mean_y += y_buffer[i];
                mean_z += z_buffer[i];
            }

            mean_x /= WINDOW_SIZE;
            mean_y /= WINDOW_SIZE;
            mean_z /= WINDOW_SIZE;

            ESP_LOGI(TAG, "Window mean -> X: %.3f Y: %.3f Z: %.3f", mean_x, mean_y, mean_z);

            // compare the mean with thresholds
            if (mean_z < 0.3f && mean_y > 0.4f)
            {
                ESP_LOGI(TAG, "Mean is over thresholds, something is happening!");
            }

            for (int i = 1; i < WINDOW_SIZE; i++)
            {
                x_buffer[i - 1] = x_buffer[i];
                y_buffer[i - 1] = y_buffer[i];
                z_buffer[i - 1] = z_buffer[i];
            }

            index = WINDOW_SIZE - 1;        
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
```