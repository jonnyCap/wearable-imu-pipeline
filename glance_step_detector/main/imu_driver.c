#include "imu_driver.h"

#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "imu_driver";

#define I2C_MASTER_SCL_IO          22
#define I2C_MASTER_SDA_IO          21
#define I2C_MASTER_NUM             I2C_NUM_0
#define I2C_MASTER_FREQ_HZ         400000
#define AXP192_ADDR                0x34

static i2c_master_bus_handle_t bus_handle = NULL;
static i2c_master_dev_handle_t dev_handle = NULL;
static i2c_master_dev_handle_t axp_dev_handle = NULL;

static esp_err_t axp_write_register(uint8_t reg, uint8_t value)
{
    uint8_t payload[2] = {reg, value};
    return i2c_master_transmit(axp_dev_handle, payload, sizeof(payload), -1);
}

static esp_err_t axp_read_register(uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(axp_dev_handle, &reg, 1, value, 1, -1);
}

static esp_err_t axp192_enable_power_rails(void)
{
    esp_err_t ret = ESP_OK;
    uint8_t value = 0;

    ret = axp_write_register(0x28, 0xCC);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_write_register(0x82, 0xFF);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_write_register(0x33, 0xC0);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_read_register(0x12, &value);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_write_register(0x12, value | 0x4D);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_write_register(0x36, 0x0C);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_write_register(0x91, 0xF0);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_write_register(0x90, 0x02);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_write_register(0x30, 0x80);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_write_register(0x39, 0xFC);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_write_register(0x35, 0xA2);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = axp_write_register(0x32, 0x46);
    return ret;
}

esp_err_t imu_init(void)
{
    esp_err_t ret = ESP_OK;

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

    i2c_device_config_t axp_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = AXP192_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };

    ret = i2c_master_bus_add_device(bus_handle, &axp_cfg, &axp_dev_handle);
    if (ret == ESP_OK) {
        ret = axp192_enable_power_rails();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "AXP192 power setup failed: %s", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "AXP192 power rails enabled");
        }
    } else {
        ESP_LOGW(TAG, "AXP192 not available on I2C bus: %s", esp_err_to_name(ret));
        axp_dev_handle = NULL;
        ret = ESP_OK;
    }

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

    uint8_t whoami = 0;
    uint8_t whoami_register = MPU6050_REG_WHOAMI;
    ret = i2c_master_transmit_receive(dev_handle, &whoami_register, 1, &whoami, 1, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read WHO_AM_I register: %s", esp_err_to_name(ret));
        return ret;
    }

    if (whoami != 0x68 && whoami != 0x19) {
        ESP_LOGW(TAG, "Unexpected WHO_AM_I value: 0x%02X", whoami);
    }

    uint8_t init_cmds[][2] = {
        {MPU6050_REG_PWR_MGMT_1, 0x00},
        {MPU6886_REG_ACCEL_CONFIG, 0x00},
        {MPU6886_REG_ACCEL_CONFIG_2, 0x01},
        {MPU6886_REG_SMPLRT_DIV, 0x05},
    };

    for (size_t i = 0; i < sizeof(init_cmds) / sizeof(init_cmds[0]); ++i) {
        ret = i2c_master_transmit(dev_handle, init_cmds[i], sizeof(init_cmds[i]), -1);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initialize MPU sensor register 0x%02X: %s",
                     init_cmds[i][0], esp_err_to_name(ret));
            return ret;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(20));

    ESP_LOGI(TAG, "MPU6050 initialized successfully");
    return ESP_OK;
}

esp_err_t imu_read_accel(float *x, float *y, float *z)
{
    if (!dev_handle || !x || !y || !z) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t accel_data[6];
    uint8_t reg = MPU6050_REG_ACCEL_XOUT_H;

    esp_err_t ret = i2c_master_transmit_receive(dev_handle, &reg, 1, accel_data, 6, -1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read accelerometer data: %s", esp_err_to_name(ret));
        return ret;
    }

    int16_t accel_x_raw = (int16_t)((accel_data[0] << 8) | accel_data[1]);
    int16_t accel_y_raw = (int16_t)((accel_data[2] << 8) | accel_data[3]);
    int16_t accel_z_raw = (int16_t)((accel_data[4] << 8) | accel_data[5]);

    *x = accel_x_raw / MPU6050_ACCEL_SENSITIVITY;
    *y = accel_y_raw / MPU6050_ACCEL_SENSITIVITY;
    *z = accel_z_raw / MPU6050_ACCEL_SENSITIVITY;

    return ESP_OK;
}

esp_err_t imu_deinit(void)
{
    esp_err_t ret = ESP_OK;

    if (dev_handle) {
        ret = i2c_master_bus_rm_device(dev_handle);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove device: %s", esp_err_to_name(ret));
        }
        dev_handle = NULL;
    }

    if (axp_dev_handle) {
        esp_err_t axp_ret = i2c_master_bus_rm_device(axp_dev_handle);
        if (axp_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to remove AXP192 device: %s", esp_err_to_name(axp_ret));
            ret = axp_ret;
        }
        axp_dev_handle = NULL;
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