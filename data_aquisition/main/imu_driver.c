#include "imu_driver.h"
#include "driver/i2c_master.h"
#include "esp_log.h"

static const char *TAG = "imu_driver";

// I2C configuration for M5Stack
#define I2C_MASTER_SCL_IO          22    // GPIO number for I2C clock
#define I2C_MASTER_SDA_IO          21    // GPIO number for I2C data
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         400000  // 400 kHz

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;

esp_err_t imu_init(void) {
    esp_err_t ret = ESP_OK;
    
    // Configure I2C master
    i2c_master_bus_config_t i2c_mst_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_MASTER_NUM,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    ret = i2c_new_master_bus(&i2c_mst_config, &bus_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Configure MPU6050 device on the I2C bus
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add MPU6050 device: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Verify MPU6050 is present by reading WHO_AM_I register
    uint8_t whoami = 0;
    ret = i2c_master_transmit_receive(dev_handle, (uint8_t *)&(uint8_t){MPU6050_REG_WHOAMI}, 1, &whoami, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        return ret;
    }
    
    if (whoami != 0x68 && whoami != 0x68) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I value: 0x%02X (expected 0x68)", whoami);
    }
    
    // Wake up MPU6050 (clear sleep bit in PWR_MGMT_1 register)
    uint8_t pwr_mgmt_data[2] = {MPU6050_REG_PWR_MGMT_1, 0x00};
    ret = i2c_master_transmit(dev_handle, pwr_mgmt_data, 2, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake up MPU6050: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t imu_read_accel(float *x, float *y, float *z) {
    if (!dev_handle || !x || !y || !z) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // Read 6 bytes of accelerometer data (X, Y, Z - 2 bytes each)
    uint8_t accel_data[6];
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;
    
    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, accel_data, 6, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // Convert raw 16-bit values to proper acceleration values
    // Each acceleration value is 16-bit (MSB first)
    int16_t accel_x_raw = (int16_t)((accel_data[0] << 8) | accel_data[1]);
    int16_t accel_y_raw = (int16_t)((accel_data[2] << 8) | accel_data[3]);
    int16_t accel_z_raw = (int16_t)((accel_data[4] << 8) | accel_data[5]);
    
    // Convert raw values to g (gravitational acceleration)
    // For ±2g range: sensitivity = 16384 LSB/g
    *x = accel_x_raw / MPU6050_ACCEL_SENSITIVITY;
    *y = accel_y_raw / MPU6050_ACCEL_SENSITIVITY;
    *z = accel_z_raw / MPU6050_ACCEL_SENSITIVITY;
    
    return ESP_OK;
}

esp_err_t imu_deinit(void) {
    esp_err_t ret = ESP_OK;
    
    if (dev_handle) {
        ret = i2c_master_bus_rm_device(dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove device: %s", esp_err_to_name(ret));
        }
        dev_handle = NULL;
    }
    
    if (bus_handle) {
        ret = i2c_del_master_bus(bus_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to delete I2C master bus: %s", esp_err_to_name(ret));
        }
        bus_handle = NULL;
    }
    
    return ret;
}
