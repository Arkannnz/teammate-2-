#ifndef PID_H
#define PID_H

typedef struct {
    float Kp;
    float Ki;
    float Kd;

    float prev_error;
    float integral;
    float out_min;
    float out_max;
} PID_t;

void  PID_Init(PID_t *pid, float kp, float ki, float kd, float out_min, float out_max);
float PID_Compute(PID_t *pid, float setpoint, float measurement, float dt);
void  PID_Reset(PID_t *pid);

#endif /* PID_H */
