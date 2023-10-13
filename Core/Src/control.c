
/*
 * control.c
 *
 *  Created on: Oct 1, 2023
 *      Author: tantr
 */
#include "main.h"
#include "control.h"
#include "stm32f1xx_hal.h"


extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim2;


int32_t constrain(int32_t value, int32_t min, int32_t max) {
    if (value < min) {
        return min;
    } else if (value > max) {
        return max;
    } else {
        return value;
    }
}

// adjust preiod for timer
void adjustPreiodTM1(int preiod)
{
	  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	  TIM_MasterConfigTypeDef sMasterConfig = {0};

	  /* USER CODE BEGIN TIM1_Init 1 */
	  /* USER CODE END TIM1_Init 1 */
	  htim1.Instance = TIM1;
	  htim1.Init.Prescaler = 36;
	  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	  htim1.Init.Period = preiod;
	  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  htim1.Init.RepetitionCounter = 0;
	  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
void adjustPreiodTM2(int preiod)
{

//  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
//  TIM_MasterConfigTypeDef sMasterConfig = {0};
//  htim2.Init.Period = preiod;
//  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;


  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 36;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = preiod;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
}


// Control functions (PID controls, Steppers control...)

// PD controller implementation(Proportional, derivative). DT in seconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint - input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-1).
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  float Kd_setPoint = constrain((setPoint - setPointOld), -8, 8); // We limit the input part...
  output = Kp * error + (Kd * Kd_setPoint - Kd * (input - PID_errorOld)) / DT;
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  //PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return (output);
}


// PI controller implementation (Proportional, integral). DT in seconds
float speedPIControl(float DT, int16_t input, int16_t setPoint,  float Kp, float Ki)
{
  int16_t error;
  float output;

  error = setPoint - input;
  PID_errorSum += constrain(error, -ITERM_MAX_ERROR, ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum, -ITERM_MAX, ITERM_MAX);

  //Serial.println(PID_errorSum);

  output = Kp * error + Ki * PID_errorSum * DT; // DT is in miliseconds...
  return (output);
}


float positionPDControl(long actualPos, long setPointPos, float Kpp, float Kdp, int16_t speedM)
{
  float output;
  float P;

  P = constrain(Kpp*(float)(setPointPos - actualPos), -115, 115); // Limit command
  output = P + Kdp * (float)(speedM);
  return (output);
}


// Set speed of Stepper Motor1
// tspeed could be positive or negative (reverse)
void setMotorSpeedM1(int16_t tspeed)
{
  long timer_period1;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M1 - tspeed) > MAX_ACCEL)
    speed_M1 -= MAX_ACCEL;
  else if ((speed_M1 - tspeed) < -MAX_ACCEL)
    speed_M1 += MAX_ACCEL;
  else
    speed_M1 = tspeed;

  speed = speed_M1 * 25; // 1/8 Microstepping

  if (speed == 0)
  {
    timer_period1 = ZERO_SPEED;
    dir_M1 = 0;
  }
  else if (speed > 0)
  {
    timer_period1 = 2000000 / speed; // 2Mhz timer
    dir_M1 = 1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);  // DIR Motor 1 (Forward)
  }
  else
  {
    timer_period1 = 2000000 / -speed;
    dir_M1 = -1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); // Dir Motor 1
  }
  if (timer_period1 > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period1 = ZERO_SPEED;

  adjustPreiodTM1(timer_period1);
  if (__HAL_TIM_GET_COUNTER(&htim1) > htim1.Init.Period) {
    __HAL_TIM_SET_COUNTER(&htim1, 0);  // Reset the timer counter to 0
  }
}

// Set speed of Stepper Motor2
// tspeed could be positive or negative (reverse)
void setMotorSpeedM2(int16_t tspeed)
{
  long timer_period2;
  int16_t speed;

  // Limit max speed?

  // WE LIMIT MAX ACCELERATION of the motors
  if ((speed_M2 - tspeed) > MAX_ACCEL)
    speed_M2 -= MAX_ACCEL;
  else if ((speed_M2 - tspeed) < -MAX_ACCEL)
    speed_M2 += MAX_ACCEL;
  else
    speed_M2 = tspeed;


  speed = speed_M2 * 25; // 1/8 Microstepping

  if (speed == 0)
  {
    timer_period2 = ZERO_SPEED;
    dir_M2 = 0;
  }
  else if (speed > 0)
  {
    timer_period2 = 2000000 / speed; // 2Mhz timer
    dir_M2 = 1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);  // Dir Motor2 (Forward)
  }
  else
  {
    timer_period2 = 2000000 / -speed;
    dir_M2 = -1;
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_SET);  // DIR Motor 2
  }
  if (timer_period2 > 65535)   // Check for minimun speed (maximun period without overflow)
    timer_period2 = ZERO_SPEED;

  adjustPreiodTM2(timer_period2);
  // Check  if we need to reset the timer...
  if (__HAL_TIM_GET_COUNTER(&htim2) > htim2.Init.Period) {
    __HAL_TIM_SET_COUNTER(&htim2, 0);  // Reset the timer counter to 0
  }
}
