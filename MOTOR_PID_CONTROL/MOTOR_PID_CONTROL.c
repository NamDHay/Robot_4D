/*
 * MOTOR_PID_CONTROL.c
 *
 *  Created on: Nov 28, 2024
 *      Author: namdhay
 */
#include "MOTOR_PID_CONTROL.h"
void MOTOR_setPIDVelocity(MOTOR_t *motor, float Kp, float Ki, float Kd,
		float Ts) {
	PID_init(&motor->PIDVelocity, Kp, Ki, Kd, Ts);
}
void MOTOR_setPIDPosition(MOTOR_t *motor, float Kp, float Ki, float Kd,
		float Ts) {
	PID_init(&motor->PIDPosition, Kp, Ki, Kd, Ts);
}
void MOTOR_setOutputRange(MOTOR_t *motor, float OutMin, float OutMax) {
	PID_setOutputRange(&motor->PIDPosition, -900, 900);
	PID_setOutputRange(&motor->PIDVelocity, OutMin, OutMax);
}
void MOTOR_setWindupRange(MOTOR_t *motor, float OutMin, float OutMax) {
	PID_setWindupGain(&motor->PIDVelocity, 0);
	PID_setWindupRange(&motor->PIDVelocity, OutMin, OutMax);
}
void MOTOR_setAngle(MOTOR_t *motor, float setAngle) {
	motor->setPoint = setAngle * motor->ratioJoint;
}
void MOTOR_runAngle(MOTOR_t *motor) {
//    if (motor->setPoint != motor->preSetPoint)
//    {
//        PID_clear(&motor->PIDPosition);
//        PID_clear(&motor->PIDVelocity);
//        motor->preSetPoint = motor->setPoint;
//    }

	MOTOR_driver_readPosAndSpeed(motor->motorDriver, &motor->pos, &motor->speed,
			motor->PIDPosition.Sample_time);
	motor->error = motor->setPoint - motor->pos;
//    motor->error = motor->setPoint;

// PD+PI controller
	PID_compute(&motor->PIDPosition, motor->error);  // tinh PD
	PID_compute(&motor->PIDVelocity, motor->PIDPosition.Output - motor->speed); // tinh PI

	if (fabs(motor->pos) < motor->limitPos) {
		MOTOR_driver_rotary(motor->motorDriver, motor->PIDVelocity.Output);
	} else {
		MOTOR_driver_rotary(motor->motorDriver, 0);
	}
}
void MOTOR_init(MOTOR_t *motor, MOTOR_DRIVER_t *motorDriver, float ratio,
		uint16_t pinSetHome, float limitPos) {
	motor->ratioJoint = ratio;
	motor->pinSetHome = 0;
	motor->setPoint = 0;
	motor->preSetPoint = 0;
	motor->error = 0;
	motor->preError = 0;
	motor->pos = 0;
	motor->speed = 0;
	motor->motorDriver = motorDriver;
	motor->limitPos = limitPos * motor->ratioJoint;
	MOTOR_driver_setupPWM(motor->motorDriver, motor->motorDriver->htimPWM,
			motor->motorDriver->PWM_CH1, motor->motorDriver->PWM_CH2);
	MOTOR_driver_setupENCODER(motor->motorDriver, motor->motorDriver->htimENC,
			motor->motorDriver->ENC_CH1, motor->motorDriver->ENC_CH2);
}
void MOTOR_reset(MOTOR_t *motor) {
	motor->pinSetHome = 0;
	motor->setPoint = 0;
	motor->preSetPoint = 0;
	motor->error = 0;
	motor->preError = 0;
	motor->pos = 0;
	motor->speed = 0;
	MOTOR_driver_reset(motor->motorDriver);
	PID_clear(&motor->PIDPosition);
	PID_clear(&motor->PIDVelocity);
}
float MOTOR_getPos(MOTOR_t *motor) {
	return motor->pos / motor->ratioJoint;
}
static float p(float p0, float pf, float tf, float v0, float vf, float T) {
	return p0 + v0 * T
			+ (3 * (pf - p0) / (tf * tf) - 2 * v0 / tf - vf / tf) * (T * T)
			+ (-2 * (pf - p0) / (tf * tf * tf) + (vf + v0) / (tf * tf))
					* (T * T * T);
}
