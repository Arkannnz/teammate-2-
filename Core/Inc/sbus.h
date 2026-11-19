#ifndef SBUS_H
#define SBUS_H

#include "main.h"

#define SBUS_FRAME_LEN      25
#define SBUS_HEADER         0x0F
#define SBUS_FOOTER         0x00
#define SBUS_CH_MIN         172
#define SBUS_CH_CENTER      992
#define SBUS_CH_MAX         1811

typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  rx_byte;
    uint8_t  frame[SBUS_FRAME_LEN];
    uint8_t  idx;
    uint32_t last_byte_ms;
    uint32_t last_frame_ms;

    uint16_t channels[16]; // Nilai mentah 16 channel (172 - 1811)
    uint8_t  frame_lost;
    uint8_t  failsafe;
} SBUS_t;

void  SBUS_Init(SBUS_t *sbus, UART_HandleTypeDef *huart);
void  SBUS_UART_RxCpltCallback(SBUS_t *sbus, UART_HandleTypeDef *huart);
uint8_t SBUS_IsConnected(SBUS_t *sbus);
float SBUS_ChannelToFloat(uint16_t raw, float out_min, float out_max);

#endif /* SBUS_H */
