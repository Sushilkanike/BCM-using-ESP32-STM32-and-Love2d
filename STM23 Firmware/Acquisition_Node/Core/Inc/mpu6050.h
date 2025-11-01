#pragma once
#include "stm32f4xx_hal.h"
#define MPU6050_ADDR (0x68 << 1)
typedef struct {
    int16_t Accel_X_RAW, Accel_Y_RAW, Accel_Z_RAW;
    int16_t Gyro_X_RAW, Gyro_Y_RAW, Gyro_Z_RAW;
    float Ax, Ay, Az, Gx, Gy, Gz;
} MPU6050_Data;
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);
