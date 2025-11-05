#include "mpu6050.h"
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t check, data;
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x75, 1, &check, 1, 1000);
    if (check != 0x68) return 0;
    data = 0; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x6B, 1, &data, 1, 1000);
    data = 0x07; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x19, 1, &data, 1, 1000);
    data = 0x00; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1C, 1, &data, 1, 1000);
    data = 0x00; HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, 0x1B, 1, &data, 1, 1000);
    return 1;
}
void MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data *data) {
    uint8_t Rec_Data[14];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3B, 1, Rec_Data, 14, 1000);
    data->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    data->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    data->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    data->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    data->Ax = data->Accel_X_RAW / 16384.0; data->Ay = data->Accel_Y_RAW / 16384.0;
    data->Az = data->Accel_Z_RAW / 16384.0; data->Gx = data->Gyro_X_RAW / 131.0;
}
