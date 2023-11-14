/*
 * control.h
 *
 *  Created on: Oct 1, 2023
 *      Author: tantr
 */

#ifndef INC_CONTROL_H_
#define INC_CONTROL_H_

#include "stm32f1xx_it.h"
//#include "stm32f1xx_hal_tim.h"
#define MAX_CONTROL_OUTPUT 500
#define ITERM_MAX_ERROR 30   // Iterm windup constants for PI control
#define ITERM_MAX 10000
#define ZERO_SPEED 65535
#define MAX_ACCEL 2

void adjustPreiodTM1(int preiod);
void adjustPreiodTM2(int preiod);
void setMotorSpeedM1(int16_t tspeed);
void setMotorSpeedM2(int16_t tspeed);
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd);
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki);
float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM);
int32_t constrain(int32_t value, int32_t min, int32_t max);

#endif /* INC_CONTROL_H_ */
