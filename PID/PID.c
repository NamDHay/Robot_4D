/*
 * PID.c
 *
 *  Created on: Nov 28, 2024
 *      Author: namdhay
 */
#include "PID.h"
#include "stdint.h"
void PID_init(PID_TypeDef *pid, float Kp, float Ki, float Kd, float Sample_time) {
	PID_clear(pid);
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Sample_time = Sample_time;
}
void PID_clear(PID_TypeDef *pid) {
	pid->PTerm = 0;
	pid->ITerm = 0;
	pid->DTerm = 0;
	pid->Output = 0;
	pid->FeedbackWindup = 0;
}
void PID_setKp(PID_TypeDef *pid, float value) {
	pid->Kp = value;
}
void PID_setKi(PID_TypeDef *pid, float value) {
	pid->Ki = value;
}
void PID_setKd(PID_TypeDef *pid, float value) {
	pid->Kd = value;
}
void PID_setSampleTime(PID_TypeDef *pid, float value) {
	pid->Sample_time = value;
}
void PID_setOutputRange(PID_TypeDef *pid, float valueMin, float valueMax) {
	pid->OutMax = valueMax;
	pid->OutMin = valueMin;
}
void PID_setWindupRange(PID_TypeDef *pid, float valueMin, float valueMax) {
	pid->WindupMax = valueMax;
	pid->WindupMin = valueMin;
}
void PID_setWindupGain(PID_TypeDef *pid, float value) {
	pid->Kb = value;
}
float PID_compute(PID_TypeDef *pid, float Error) {

	pid->PTerm = pid->Kp * Error;

	pid->ITerm += ((Error + pid->Last_error) * pid->Ki * 0.5
			+ pid->FeedbackWindup * pid->Kb) * (pid->Sample_time / 1000.0);
	if (pid->ITerm > pid->WindupMax)
		pid->ITerm = pid->WindupMax;
	else if (pid->ITerm < pid->WindupMin)
		pid->ITerm = pid->WindupMin;

	pid->Delta_error = Error - pid->Last_error;
	pid->DTerm = pid->Kd * pid->Delta_error / (pid->Sample_time / 1000.0);
	pid->Last_error = Error;

	pid->Output = pid->PTerm + pid->ITerm + pid->DTerm;
	if (pid->Output > pid->OutMax) {
		pid->FeedbackWindup = pid->Output - pid->OutMax;
		pid->Output = pid->OutMax;
	} else if (pid->Output < pid->OutMin) {
		pid->FeedbackWindup = pid->Output - pid->OutMin;
		pid->Output = pid->OutMin;
	} else {
		pid->FeedbackWindup = 0;
	}
	return pid->Output;
}

