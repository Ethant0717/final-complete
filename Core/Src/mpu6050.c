/*
 * mpu6050.c
 *
 *  Created on: Apr 6, 2026
 *      Author: ethan
 */

#include "mpu6050.h"

/* ==============================
   INTERNAL STATE
============================== */
static float gyro_bias_x = 0;
static float gyro_bias_y = 0;
static float gyro_bias_z = 0;

static float gyro_filt_x = 0;
static float gyro_filt_y = 0;
static float gyro_filt_z = 0;

/* ==============================
   INIT
============================== */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data;

    /* Wake up */
    data = 0;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_PWR_MGMT_1, 1, &data, 1, HAL_MAX_DELAY);

    /* Sample rate */
    data = 0x07;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_SMPLRT_DIV, 1, &data, 1, HAL_MAX_DELAY);

    /* DLPF (important for noise!) */
    data = 0x03;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_CONFIG, 1, &data, 1, HAL_MAX_DELAY);

    /* Gyro ±250 deg/s */
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_GYRO_CONFIG, 1, &data, 1, HAL_MAX_DELAY);

    /* Accel ±2g */
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_REG_ACCEL_CONFIG, 1, &data, 1, HAL_MAX_DELAY);

    return HAL_OK;
}

/* ==============================
   CALIBRATION (KEEP STILL!)
============================== */
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c)
{
    int32_t sum_x = 0, sum_y = 0, sum_z = 0;
    uint8_t buffer[6];

    for (int i = 0; i < 500; i++)
    {
        HAL_I2C_Mem_Read(hi2c,
                         MPU6050_ADDR,
                         0x43,
                         1,
                         buffer,
                         6,
                         HAL_MAX_DELAY);

        int16_t gx = (int16_t)(buffer[0] << 8 | buffer[1]);
        int16_t gy = (int16_t)(buffer[2] << 8 | buffer[3]);
        int16_t gz = (int16_t)(buffer[4] << 8 | buffer[5]);

        sum_x += gx;
        sum_y += gy;
        sum_z += gz;

        HAL_Delay(2);
    }

    gyro_bias_x = (sum_x / 500.0f) / GYRO_SCALE;
    gyro_bias_y = (sum_y / 500.0f) / GYRO_SCALE;
    gyro_bias_z = (sum_z / 500.0f) / GYRO_SCALE;
}

/* ==============================
   READ ALL DATA
============================== */
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data)
{
    uint8_t buffer[14];

    if (HAL_I2C_Mem_Read(hi2c,
                         MPU6050_ADDR,
                         MPU6050_REG_ACCEL_XOUT_H,
                         1,
                         buffer,
                         14,
                         HAL_MAX_DELAY) != HAL_OK)
    {
        return HAL_ERROR;
    }

    /* RAW VALUES */
    int16_t raw_ax = (int16_t)(buffer[0] << 8 | buffer[1]);
    int16_t raw_ay = (int16_t)(buffer[2] << 8 | buffer[3]);
    int16_t raw_az = (int16_t)(buffer[4] << 8 | buffer[5]);

    int16_t raw_gx = (int16_t)(buffer[8] << 8 | buffer[9]);
    int16_t raw_gy = (int16_t)(buffer[10] << 8 | buffer[11]);
    int16_t raw_gz = (int16_t)(buffer[12] << 8 | buffer[13]);

    /* ACCEL (g) */
    data->Accel_X = raw_ax / ACCEL_SCALE;
    data->Accel_Y = raw_ay / ACCEL_SCALE;
    data->Accel_Z = raw_az / ACCEL_SCALE;

    /* GYRO (deg/s) */
    float gx = (raw_gx / GYRO_SCALE) - gyro_bias_x;
    float gy = (raw_gy / GYRO_SCALE) - gyro_bias_y;
    float gz = (raw_gz / GYRO_SCALE) - gyro_bias_z;

    /* LOW PASS FILTER */
    gyro_filt_x = 0.7f * gyro_filt_x + 0.3f * gx;
    gyro_filt_y = 0.7f * gyro_filt_y + 0.3f * gy;
    gyro_filt_z = 0.7f * gyro_filt_z + 0.3f * gz;

    data->Gyro_X = gyro_filt_x;
    data->Gyro_Y = gyro_filt_y;
    data->Gyro_Z = gyro_filt_z;

    return HAL_OK;
}

/* ==============================
   WHO AM I
============================== */
uint8_t MPU6050_WhoAmI(I2C_HandleTypeDef *hi2c)
{
    uint8_t check = 0;
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_REG_WHO_AM_I, 1, &check, 1, HAL_MAX_DELAY);
    return check;
}
