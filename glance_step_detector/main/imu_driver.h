#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include "esp_err.h"

#define MPU6050_ADDR 0x68

#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_ACCEL_XOUT_L  0x3C
#define MPU6050_REG_ACCEL_YOUT_H  0x3D
#define MPU6050_REG_ACCEL_YOUT_L  0x3E
#define MPU6050_REG_ACCEL_ZOUT_H  0x3F
#define MPU6050_REG_ACCEL_ZOUT_L  0x40
#define MPU6886_REG_SMPLRT_DIV    0x19
#define MPU6886_REG_CONFIG        0x1A
#define MPU6886_REG_ACCEL_CONFIG  0x1C
#define MPU6886_REG_ACCEL_CONFIG_2 0x1D
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHOAMI        0x75

#define MPU6050_ACCEL_SENSITIVITY 16384.0f

/**
 * @brief Initialize the IMU driver and sensor hardware.
 *
 * This routine creates an ESP-IDF I2C master bus, optionally configures the
 * AXP192 power-management IC (if present on the board), attaches the MPU60x0/
 * MPU6886-compatible accelerometer device, validates communication via
 * WHO_AM_I, and writes the startup configuration needed for accelerometer
 * sampling.
 *
 * @return
 * - ESP_OK on successful initialization and sensor configuration.
 * - ESP_ERR_* code from I2C, power-rail setup, or sensor register writes if
 *   initialization fails.
 */
esp_err_t imu_init(void);

/**
 * @brief Read one accelerometer sample in units of g.
 *
 * Reads the 6-byte accelerometer output register block starting at
 * MPU6050_REG_ACCEL_XOUT_H, converts the raw signed 16-bit axis values to
 * floating-point acceleration using MPU6050_ACCEL_SENSITIVITY, and stores the
 * result in the caller-provided output pointers.
 *
 * @param[out] x Pointer to receive acceleration on the X axis in g.
 * @param[out] y Pointer to receive acceleration on the Y axis in g.
 * @param[out] z Pointer to receive acceleration on the Z axis in g.
 *
 * @return
 * - ESP_OK when a complete sample is read and converted.
 * - ESP_ERR_INVALID_ARG if the driver is not initialized or any output pointer
 *   is NULL.
 * - ESP_ERR_* code returned by the I2C read transaction on communication
 *   failure.
 */
esp_err_t imu_read_accel(float *x, float *y, float *z);

/**
 * @brief Deinitialize IMU and I2C resources.
 *
 * Removes all registered I2C devices (MPU and optional AXP192) and deletes the
 * underlying I2C master bus. Internal handles are cleared regardless of
 * success so repeated cleanup calls remain safe.
 *
 * @return
 * - ESP_OK when all resources are released successfully.
 * - ESP_ERR_* code if any device removal or bus deletion operation fails.
 */
esp_err_t imu_deinit(void);

#endif