#ifndef MPU6050_H_
#define MPU6050_H_

#include "stm32l4xx_hal.h"

/* ==============================
   DEVICE ADDRESS
============================== */
#define MPU6050_ADDR (0x69 << 1)

/* ==============================
   REGISTERS
============================== */
#define MPU6050_REG_PWR_MGMT_1     0x6B
#define MPU6050_REG_SMPLRT_DIV     0x19
#define MPU6050_REG_CONFIG         0x1A
#define MPU6050_REG_GYRO_CONFIG    0x1B
#define MPU6050_REG_ACCEL_CONFIG   0x1C
#define MPU6050_REG_WHO_AM_I       0x75
#define MPU6050_REG_ACCEL_XOUT_H   0x3B

/* ==============================
   SCALE FACTORS
============================== */
#define ACCEL_SCALE 16384.0f
#define GYRO_SCALE  131.0f

/* ==============================
   DATA STRUCT
============================== */
typedef struct
{
    float Accel_X;
    float Accel_Y;
    float Accel_Z;

    float Gyro_X;
    float Gyro_Y;
    float Gyro_Z;

} MPU6050_Data_t;

/* ==============================
   FUNCTIONS
============================== */
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *data);
uint8_t MPU6050_WhoAmI(I2C_HandleTypeDef *hi2c);

#endif
