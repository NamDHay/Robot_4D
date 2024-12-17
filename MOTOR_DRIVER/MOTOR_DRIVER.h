/*
 * MOTOR_DRIVER.h
 *
 *  Created on: Nov 28, 2024
 *      Author: namdhay
 */

#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_
#include "stm32f4xx_hal.h"

typedef struct {
	TIM_HandleTypeDef *htimENC;
	TIM_HandleTypeDef *htimPWM;
	int64_t EncCount;
	uint32_t PWM_CH1;
	uint32_t PWM_CH2;
	uint32_t ENC_CH1;
	uint32_t ENC_CH2;
	float speed;
	float pos;
	float preSpeed;
	float prePos;
	float ratio;
} MOTOR_DRIVER_t;
void MOTOR_driver_setupPWM(MOTOR_DRIVER_t *motor, TIM_HandleTypeDef *htimPWM,
		uint32_t CH1, uint32_t CH2);
void MOTOR_driver_setupENCODER(MOTOR_DRIVER_t *motor,
		TIM_HandleTypeDef *htimENC, uint32_t CH1, uint32_t CH2);
void MOTOR_driver_setRatio(MOTOR_DRIVER_t *motor, float ratio);
void MOTOR_driver_reset(MOTOR_DRIVER_t *motor);
void MOTOR_driver_readPosAndSpeed(MOTOR_DRIVER_t *motor, float *pos,
		float *speed, float Ts);
void MOTOR_driver_rotary(MOTOR_DRIVER_t *motor, float duty);
#endif /* MOTOR_DRIVER_H_ */
