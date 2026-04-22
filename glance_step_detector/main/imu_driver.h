#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>

#include "esp_err.h"

#define MPU6050_ADDR 0x68

#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_ACCEL_XOUT_L  0x3C
#define MPU6050_REG_ACCEL_YOUT_H  0x3D
#define MPU6050_REG_ACCEL_YOUT_L  0x3E
#define MPU6050_REG_ACCEL_ZOUT_H  0x3F
#define MPU6050_REG_ACCEL_ZOUT_L  0x40
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_WHOAMI        0x75

#define MPU6050_ACCEL_SENSITIVITY 16384.0f

esp_err_t imu_init(void);
esp_err_t imu_read_accel(float *x, float *y, float *z);
esp_err_t imu_deinit(void);

#endif