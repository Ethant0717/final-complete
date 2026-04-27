/*
 * motors.c
 *
 *  Created on: Apr 6, 2026
 *      Author: ethan
 */
#include "control.h"
#include "motors.h"
#include <math.h>


/* =======================================================
   INIT
======================================================= */
void Motors_Init(TIM_HandleTypeDef *htim)
{
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_3);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);

    Motors_Disarm(htim);
}

/* =======================================================
   WRITE PWM TO ESCs
======================================================= */
void Motors_Write(TIM_HandleTypeDef *htim, Motors_t *m)
{
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, m->m1);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, m->m2);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, m->m3);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, m->m4);


}

/* =======================================================
   MOTOR MIXING (QUAD X)
======================================================= */
void Motors_Mix(Motors_t *m,
                float throttle,
                float roll,
                float pitch,
                float yaw)
{
    /* =====================================================
       1. BASE THROTTLE (must be strong enough to fly)
    ===================================================== */
    float base = throttle;


    float k = 10.0f;   // << THIS is what makes drone respond

    roll  *= k;
    pitch *= k;
    yaw   *= k;
    /* =====================================================
       3. MIXING EQUATIONS
    ===================================================== */
    float m1 = base + pitch - roll + yaw;
    float m2 = base + pitch + roll - yaw;
    float m3 = base - pitch + roll + yaw;
    float m4 = base - pitch - roll - yaw;


    /* =====================================================
       4. CLAMP RAW VALUES FIRST (soft safety)
    ===================================================== */
    m1 = fminf(fmaxf(m1, 1000), 2000);
    m2 = fminf(fmaxf(m2, 1000), 2000);
    m3 = fminf(fmaxf(m3, 1000), 2000);
    m4 = fminf(fmaxf(m4, 1000), 2000);


    /* =====================================================
       4.5 MOTOR BALANCE SHIFT (🔴 ADD THIS HERE)
    ===================================================== */
    float minVal = fminf(fminf(m1, m2), fminf(m3, m4));

    if (minVal < 1000)
    {
        float shift = 1000 - minVal;

        m1 += shift;
        m2 += shift;
        m3 += shift;
        m4 += shift;
    }


    /* =====================================================
       5. FINAL CLAMP (VERY IMPORTANT)
    ===================================================== */
    m1 = fminf(fmaxf(m1, 1000), 2000);
    m2 = fminf(fmaxf(m2, 1000), 2000);
    m3 = fminf(fmaxf(m3, 1000), 2000);
    m4 = fminf(fmaxf(m4, 1000), 2000);


    /* =====================================================
       6. WRITE OUTPUT
    ===================================================== */
    m->m1 = (uint16_t)m1;
    m->m2 = (uint16_t)m2;
    m->m3 = (uint16_t)m3;
    m->m4 = (uint16_t)m4;
}

//    /* =====================================================
//       3. MIXING EQUATIONS
//    ===================================================== */
//    float m1 = base + pitch - roll + yaw;
//    float m2 = base + pitch + roll - yaw;
//    float m3 = base - pitch + roll + yaw;
//    float m4 = base - pitch - roll - yaw;
//
//    /* =====================================================
//       4. CLAMP TO ESC RANGE
//    ===================================================== */
//    m1 = fminf(fmaxf(m1, 1000), 2000);
//    m2 = fminf(fmaxf(m2, 1000), 2000);
//    m3 = fminf(fmaxf(m3, 1000), 2000);
//    m4 = fminf(fmaxf(m4, 1000), 2000);
//
//    /* =====================================================
//       5. WRITE OUTPUT
//    ===================================================== */
//    m->m1 = (uint16_t)m1;
//    m->m2 = (uint16_t)m2;
//    m->m3 = (uint16_t)m3;
//    m->m4 = (uint16_t)m4;
//}



/* =======================================================
   ARM / DISARM
======================================================= */
void Motors_Arm(TIM_HandleTypeDef *htim)
{
    // Send minimum throttle for ESC arming
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, MOTOR_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, MOTOR_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, MOTOR_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, MOTOR_MIN);

    HAL_Delay(3000); // wait for ESC beeps
}

void Motors_Disarm(TIM_HandleTypeDef *htim)
{
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, MOTOR_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, MOTOR_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_3, MOTOR_MIN);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_4, MOTOR_MIN);
}
