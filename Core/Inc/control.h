/*
 * control.h
 *
 *  Created on: Apr 6, 2026
 *      Author: ethan
 */

#ifndef CONTROL_H
#define CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif





#include "stm32l4xx_hal.h"
#include <stdint.h>
#include "mpu6050.h"
/* ---------------- Constants ---------------- */
#define PWM_MIN    1000
#define PWM_MAX    2000
#define PWM_IDLE   1100

/* ---------------- Attitude ---------------- */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
} Attitude_t;

/* ---------------- Setpoints ---------------- */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float throttle;
} Setpoint_t;
extern Setpoint_t sp;
/* ---------------- PID Controller ---------------- */
typedef struct
{
    float kp;
    float ki;
    float kd;

    float integral;
    float prev_error;

} PID_t;

/* ---------------- Control Output ---------------- */
typedef struct
{
    float roll;
    float pitch;
    float yaw;
    float throttle;
} ControlOutput_t;

/* ---------------- Functions ---------------- */

/* Init PID + control */
void Control_Init(void);

/* Convert IMU → attitude */
void Control_UpdateAttitude(MPU6050_Data_t *imu, Attitude_t *att);

/* PID compute */
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt);

/* Compute control corrections */
//void Control_Compute(Attitude_t *att, ControlOutput_t *out);
void Control_Compute(Attitude_t *att, MPU6050_Data_t *imu, ControlOutput_t *out);

/* Clamp helper */
float Control_Clamp(float value, float min, float max);

#ifdef __cplusplus
}
#endif

#endif /* CONTROL_H */
