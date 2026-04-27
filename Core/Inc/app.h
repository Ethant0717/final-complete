/*
 * app.h
 *
 *  Created on: Apr 6, 2026
 *      Author: ethan
 */

#ifndef APP_H
#define APP_H

#include "main.h"
#include "control.h"
#include "motors.h"
extern UART_HandleTypeDef huart2;
/* Core functions */
void App_Init(void);
void App_Update(void);

#endif
