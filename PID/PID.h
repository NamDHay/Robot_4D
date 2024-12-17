/*
 * PID.h
 *
 *  Created on: Nov 28, 2024
 *      Author: namdhay
 */

#ifndef PID_H_
#define PID_H_

typedef struct PID {
	volatile float Kp;
	volatile float Ki;
	volatile float Kd;
	volatile float Kb;
	volatile float SetPoint;
	volatile float Last_error;
	volatile float Windup_guard;
	volatile float PTerm;
	volatile float ITerm;
	volatile float DTerm;
	volatile float WindupMax;
	volatile float WindupMin;
	volatile float FeedbackWindup;
	volatile float Delta_error;
	volatile float Output;
	volatile float OutMax;
	volatile float OutMin;
	volatile float Sample_time;

} PID_TypeDef;
void PID_init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float Sample_time);
void PID_clear(PID_TypeDef *pid);
void PID_setKp(PID_TypeDef *pid, float value);
void PID_setKi(PID_TypeDef *pid, float value);
void PID_setKd(PID_TypeDef *pid, float value);
void PID_setSampleTime(PID_TypeDef *pid, float value);
void PID_setOutputRange(PID_TypeDef *pid, float valueMin, float valueMax);
void PID_setWindupRange(PID_TypeDef *pid, float valueMin, float valueMax);
void PID_setWindupGain(PID_TypeDef *pid, float value);
float PID_compute(PID_TypeDef *pid, float Error);

#endif /* PID_H_ */
