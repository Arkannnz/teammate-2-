

#include "pid.h"

void PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
    pid->out_min = out_min;
    pid->out_max = out_max;
}

float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt)
{
    if (dt <= 0.0f) {
        return 0.0f;
    }

    float error = setpoint - measurement;

    // Proporsional
    float P = pid->Kp * error;

    // Integral dengan anti-windup
    pid->integral += error * dt;
    if (pid->Ki > 0.0f) {
        float max_i = pid->out_max / pid->Ki;
        float min_i = pid->out_min / pid->Ki;
        if (pid->integral > max_i) pid->integral = max_i;
        if (pid->integral < min_i) pid->integral = min_i;
    }
    float I = pid->Ki * pid->integral;

    // Derivatif
    float derivative = (error - pid->prev_error) / dt;
    float D = pid->Kd * derivative;
    pid->prev_error = error;

    // Total output dengan batas aman
    float output = P + I + D;
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    return output;
}

void PID_Reset(PID_t *pid)
{
    pid->prev_error = 0.0f;
    pid->integral = 0.0f;
}
