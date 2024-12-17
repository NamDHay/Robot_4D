/*
 * MOTOR_PID_CONTROL.h
 *
 *  Created on: Nov 28, 2024
 *      Author: namdhay
 */

#ifndef MOTOR_PID_CONTROL_H_
#define MOTOR_PID_CONTROL_H_

#include "MOTOR_DRIVER.h"
#include "PID.h"
#include "math.h"
typedef struct {
	PID_TypeDef PIDVelocity;
	PID_TypeDef PIDPosition;
	MOTOR_DRIVER_t *motorDriver;
	uint16_t pinSetHome;
	float setPoint;
	float preSetPoint;
	float error;
	float preError;
	float ratioJoint;
	float pos;
	float speed;
	float limitPos;
} MOTOR_t;
void MOTOR_setPIDVelocity(MOTOR_t *motor, float Kp, float Ki, float Kd,
		float Ts);
void MOTOR_setPIDPosition(MOTOR_t *motor, float Kp, float Ki, float Kd,
		float Ts);
void MOTOR_setAngle(MOTOR_t *motor, float setAngle);
void MOTOR_setWindupRange(MOTOR_t *motor, float OutMin, float OutMax);
void MOTOR_setOutputRange(MOTOR_t *motor, float OutMin, float OutMax);
void MOTOR_runAngle(MOTOR_t *motor);
void MOTOR_init(MOTOR_t *motor, MOTOR_DRIVER_t *motorDriver, float ratio,
		uint16_t pinSetHome, float limitPos);
float MOTOR_getPos(MOTOR_t *motor);
static float p(float p0, float pf, float tf, float v0, float vf, float T);
#endif /* MOTOR_PID_CONTROL_H_ */
