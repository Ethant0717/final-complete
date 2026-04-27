/*
 * control.c
 */

#include "control.h"
#include "math.h"
#include "control.h"
#include "math.h"
#include <stdio.h>

/* =======================================================
   PID CONTROLLERS (RATE LOOP ONLY)
======================================================= */
static PID_t pid_rate_roll  = {0.12f, 0.0f, 0.002f, 0.0f, 0.0f};
static PID_t pid_rate_pitch = {0.12f, 0.0f, 0.002f, 0.0f, 0.0f};
static PID_t pid_rate_yaw   = {0.20f, 0.0f, 0.000f, 0.0f, 0.0f};

/* =======================================================
   TIMING
======================================================= */
static uint32_t lastTick = 0;
static uint32_t lastPIDTick = 0;

/* =======================================================
   YAW BIAS CALIBRATION
======================================================= */
static float gyroZ_bias = 0.0f;
static uint8_t yaw_calibrated = 0;

/* GLOBAL SETPOINT */
Setpoint_t sp;

/* =======================================================
   INIT
======================================================= */
void Control_Init(void)
{
    /* RESET RATE PID STATES */
    pid_rate_roll.integral = 0.0f;
    pid_rate_pitch.integral = 0.0f;
    pid_rate_yaw.integral = 0.0f;

    pid_rate_roll.prev_error = 0.0f;
    pid_rate_pitch.prev_error = 0.0f;
    pid_rate_yaw.prev_error = 0.0f;

    lastTick = HAL_GetTick();
    lastPIDTick = lastTick;

    gyroZ_bias = 0.0f;
    yaw_calibrated = 0;
}

/* =======================================================
   ATTITUDE ESTIMATION (COMPLEMENTARY FILTER)
======================================================= */
void Control_UpdateAttitude(MPU6050_Data_t *imu, Attitude_t *att)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - lastTick) * 0.001f;
    lastTick = now;

    if (dt <= 0.0f || dt > 0.02f)
        dt = 0.002f;

    /* ACCEL ANGLES */
    float accel_roll  = atan2f(imu->Accel_X, imu->Accel_Z) * 57.2958f;
    float accel_pitch = atan2f(-imu->Accel_Y, imu->Accel_Z) * 57.2958f;

    /* GYRO BIAS CALIBRATION */
    if (!yaw_calibrated)
    {
        static uint16_t count = 0;
        static float sum = 0.0f;

        sum += imu->Gyro_Z;
        count++;

        if (count >= 500)
        {
            gyroZ_bias = sum / 500.0f;
            yaw_calibrated = 1;
        }
    }

    float gz = imu->Gyro_Z - gyroZ_bias;

    /* COMPLEMENTARY FILTER */

    att->roll  = 0.98f * (att->roll  + imu->Gyro_X * dt) + 0.02f * accel_roll;
    att->pitch = 0.98f * (att->pitch + imu->Gyro_Y * dt) + 0.02f * accel_pitch;


    /* YAW */
    att->yaw += gz * dt;

    if (att->yaw > 180.0f) att->yaw -= 360.0f;
    if (att->yaw < -180.0f) att->yaw += 360.0f;
}

/* =======================================================
   PID UPDATE
======================================================= */
float PID_Update(PID_t *pid, float setpoint, float measurement, float dt)
{
    float error = setpoint - measurement;

    pid->integral += error * dt;

    /* ANTI-WINDUP */
    if (pid->integral > 100.0f) pid->integral = 100.0f;
    if (pid->integral < -100.0f) pid->integral = -100.0f;

    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;

    return (pid->kp * error) +
           (pid->ki * pid->integral) +
           (pid->kd * derivative);
}

/* =======================================================
   MAIN CONTROL (CASCADE PID)
======================================================= */
void Control_Compute(Attitude_t *att, MPU6050_Data_t *imu, ControlOutput_t *out)
{
    uint32_t now = HAL_GetTick();
    float dt = (now - lastPIDTick) * 0.001f;
    lastPIDTick = now;

    if (dt <= 0.0f || dt > 0.02f)
        dt = 0.002f;

    /* =========================================
       OUTER LOOP (ANGLE → RATE)
    ========================================= */
    float roll_error  = sp.roll  - att->roll;
    float pitch_error = sp.pitch - att->pitch;
    float yaw_error   = sp.yaw   - att->yaw;

    /* DEADZONE (removes ESP32 noise) */
    if (fabsf(roll_error) < 1.0f)  roll_error = 0;
    if (fabsf(pitch_error) < 1.0f) pitch_error = 0;
    if (fabsf(yaw_error) < 1.0f)   yaw_error = 0;

    /* RATE SETPOINT LIMITING */
    float roll_rate_sp  = fmaxf(fminf(4.0f * roll_error,  200.0f), -200.0f);
    float pitch_rate_sp = fmaxf(fminf(4.0f * pitch_error, 200.0f), -200.0f);
    float yaw_rate_sp   = fmaxf(fminf(3.0f * yaw_error,   150.0f), -150.0f);
    printf("roll_rate_sp: %.2f imuX: %.2f\n", roll_rate_sp, imu->Gyro_X);


    /* =========================================
       INNER LOOP (RATE PID)
    ========================================= */
    float roll_cmd = PID_Update(&pid_rate_roll,
                               roll_rate_sp,
                               imu->Gyro_X,
                               dt);

    float pitch_cmd = PID_Update(&pid_rate_pitch,
                                pitch_rate_sp,
                                imu->Gyro_Y,
                                dt);

    float yaw_cmd = PID_Update(&pid_rate_yaw,
                              yaw_rate_sp,
                              imu->Gyro_Z - gyroZ_bias,
                              dt);

    /* LIMIT */
    const float LIMIT = 200.0f;

    roll_cmd  = fmaxf(fminf(roll_cmd,  LIMIT), -LIMIT);
    pitch_cmd = fmaxf(fminf(pitch_cmd, LIMIT), -LIMIT);
    yaw_cmd   = fmaxf(fminf(yaw_cmd,   LIMIT), -LIMIT);

    /* OUTPUT */
    out->roll  = roll_cmd;
    out->pitch = pitch_cmd;
    out->yaw   = yaw_cmd;
    out->throttle = sp.throttle;
}



//void Control_Compute(Attitude_t *att, ControlOutput_t *out)
//{
//    /* ===================================================
//       TIME STEP
//    =================================================== */
//    uint32_t now = HAL_GetTick();
//    float dt = (now - lastPIDTick) * 0.001f;
//    lastPIDTick = now;
//
//    if (dt <= 0.0f || dt > 0.02f)
//        dt = 0.002f;
//
//    /* ===================================================
//       PID COMPUTATION (ANGLE ERROR)
//    =================================================== */
//    float roll_cmd  = PID_Update(&pid_roll,  sp.roll,  att->roll,  dt);
//    float pitch_cmd = PID_Update(&pid_pitch, sp.pitch, att->pitch, dt);
//    float yaw_cmd   = PID_Update(&pid_yaw,   sp.yaw,   att->yaw,   dt);
//
//    /* ===================================================
//       SOFT LIMITING (BETTER THAN HARD CLAMP)
//       prevents sudden saturation and motor spikes
//    =================================================== */
//    const float LIMIT = 300.0f;
//
//    roll_cmd  = fmaxf(fminf(roll_cmd,  LIMIT), -LIMIT);
//    pitch_cmd = fmaxf(fminf(pitch_cmd, LIMIT), -LIMIT);
//    yaw_cmd   = fmaxf(fminf(yaw_cmd,   LIMIT), -LIMIT);
//
//    /* ===================================================
//       OPTIONAL: SIMPLE RATE SMOOTHING
//       (prevents “no visible motor change” feeling)
//    =================================================== */
//    static float last_roll = 0;
//    static float last_pitch = 0;
//    static float last_yaw = 0;
//
//    float alpha = 0.9f; // smoothing factor (0 = slow, 1 = raw)
//
//    roll_cmd  = alpha * roll_cmd  + (1 - alpha) * last_roll;
//    pitch_cmd = alpha * pitch_cmd + (1 - alpha) * last_pitch;
//    yaw_cmd   = alpha * yaw_cmd   + (1 - alpha) * last_yaw;
//
//    last_roll = roll_cmd;
//    last_pitch = pitch_cmd;
//    last_yaw = yaw_cmd;
//
//    /* ===================================================
//       OUTPUT STRUCTURE
//    =================================================== */
//    out->roll  = roll_cmd;
//    out->pitch = pitch_cmd;
//    out->yaw   = yaw_cmd;
//
//    /* throttle passthrough (IMPORTANT for arming/mixer) */
//    out->throttle = sp.throttle;
//}
