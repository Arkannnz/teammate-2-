

#ifndef SERVO_H
#define SERVO_H

#include "main.h"

typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
    float current_angle;
} Servo_t;

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel);
void Servo_SetAngle(Servo_t *servo, float angle_deg);

#endif /* SERVO_H */
