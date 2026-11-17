#include "servo.h"

void Servo_Init(Servo_t *servo, TIM_HandleTypeDef *htim, uint32_t channel)
{
    servo->htim = htim;
    servo->channel = channel;
    HAL_TIM_PWM_Start(htim, channel);
    Servo_SetAngle(servo, 90.0f); // Posisi awal netral (90°)
}

void Servo_SetAngle(Servo_t *servo, float angle_deg)
{
    if (angle_deg < 0.0f)   angle_deg = 0.0f;
    if (angle_deg > 180.0f) angle_deg = 180.0f;

    servo->current_angle = angle_deg;

    // Konversi 0°..180° menjadi lebar pulsa 1000..2000 mikrodetik (50 Hz)
    uint32_t pulse_us = (uint32_t)(1000.0f + (angle_deg / 180.0f) * 1000.0f);
    __HAL_TIM_SET_COMPARE(servo->htim, servo->channel, pulse_us);
}
