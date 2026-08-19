#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>
#include "esp_err.h"

// MPU6050 I2C address
#define MPU6050_ADDR 0x68

// MPU6050 Register addresses
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_ACCEL_XOUT_L  0x3C
#define MPU6050_REG_ACCEL_YOUT_H  0x3D
#define MPU6050_REG_ACCEL_YOUT_L  0x3E
#define MPU6050_REG_ACCEL_ZOUT_H  0x3F
#define MPU6050_REG_ACCEL_ZOUT_L  0x40
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHOAMI        0x75

// Default sensitivity (16384 LSB/g for ±2g range)
#define MPU6050_ACCEL_SENSITIVITY 16384.0f

/**
 * Initialize the IMU (MPU6050) driver
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_init(void);

/**
 * Read accelerometer data from MPU6050
 * @param x Pointer to store X-axis acceleration (in g)
 * @param y Pointer to store Y-axis acceleration (in g)
 * @param z Pointer to store Z-axis acceleration (in g)
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t imu_read_accel(float *x, float *y, float *z);

/**
 * Deinitialize the IMU driver
 * @return ESP_OK on success
 */
esp_err_t imu_deinit(void);

#endif // IMU_DRIVER_H
