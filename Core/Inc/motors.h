/*
 * motors.h
 *
 *  Created on: Apr 6, 2026
 *      Author: ethan
 */

#ifndef MOTORS_H
#define MOTORS_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32l4xx_hal.h"
#include <stdint.h>


/* =======================================================
   PWM LIMITS (ESC SIGNAL RANGE)
======================================================= */
#define MOTOR_MIN   1000.0f   // minimum throttle (µs)
#define MOTOR_MAX   2000.0f   // maximum throttle (µs)
#define MOTOR_IDLE  1100.0f   // idle spin (safe arm value)

/* =======================================================
   MOTOR MAPPING (QUAD X CONFIG)
======================================================= */
/*
         FRONT
      M1      M2
        \    /
         \  /
         /  \
        /    \
      M4      M3
         REAR
*/

/* =======================================================
   MOTOR OUTPUT STRUCT
======================================================= */
typedef struct
{
    uint16_t m1;
    uint16_t m2;
    uint16_t m3;
    uint16_t m4;
} Motors_t;


/* =======================================================
   FUNCTION PROTOTYPES
======================================================= */

/* Start PWM signals on all motor channels */
void Motors_Init(TIM_HandleTypeDef *htim);

/* Write final PWM values to ESCs */
void Motors_Write(TIM_HandleTypeDef *htim, Motors_t *motors);

/* Mix roll/pitch/yaw/throttle into motor outputs */
void Motors_Mix(Motors_t *motors,
                float throttle,
                float roll,
                float pitch,
                float yaw);

/* Arm ESCs safely */
void Motors_Arm(TIM_HandleTypeDef *htim);

/* Disarm motors (set to minimum) */
void Motors_Disarm(TIM_HandleTypeDef *htim);

#ifdef __cplusplus
}
#endif

#endif /* MOTORS_H */
