#include "sbus.h"

static void SBUS_ParseFrame(SBUS_t *sbus)
{
    const uint8_t *b = sbus->frame;

    sbus->channels[0]  = ((b[1]       | b[2]  << 8) & 0x07FF);
    sbus->channels[1]  = ((b[2]  >> 3 | b[3]  << 5) & 0x07FF);
    sbus->channels[2]  = ((b[3]  >> 6 | b[4]  << 2 | b[5] << 10) & 0x07FF);
    sbus->channels[3]  = ((b[5]  >> 1 | b[6]  << 7) & 0x07FF);
    sbus->channels[4]  = ((b[6]  >> 4 | b[7]  << 4) & 0x07FF);
    sbus->channels[5]  = ((b[7]  >> 7 | b[8]  << 1 | b[9] << 9) & 0x07FF);
    sbus->channels[6]  = ((b[9]  >> 2 | b[10] << 6) & 0x07FF);
    sbus->channels[7]  = ((b[10] >> 5 | b[11] << 3) & 0x07FF);
    sbus->channels[8]  = ((b[12]      | b[13] << 8) & 0x07FF);
    sbus->channels[9]  = ((b[13] >> 3 | b[14] << 5) & 0x07FF);
    sbus->channels[10] = ((b[14] >> 6 | b[15] << 2 | b[16] << 10) & 0x07FF);
    sbus->channels[11] = ((b[16] >> 1 | b[17] << 7) & 0x07FF);
    sbus->channels[12] = ((b[17] >> 4 | b[18] << 4) & 0x07FF);
    sbus->channels[13] = ((b[18] >> 7 | b[19] << 1 | b[20] << 9) & 0x07FF);
    sbus->channels[14] = ((b[20] >> 2 | b[21] << 6) & 0x07FF);
    sbus->channels[15] = ((b[21] >> 5 | b[22] << 3) & 0x07FF);

    sbus->frame_lost = (b[23] & (1 << 2)) ? 1 : 0;
    sbus->failsafe   = (b[23] & (1 << 3)) ? 1 : 0;
    sbus->last_frame_ms = HAL_GetTick();
}

void SBUS_Init(SBUS_t *sbus, UART_HandleTypeDef *huart)
{
    sbus->huart = huart;
    sbus->idx = 0;
    sbus->last_byte_ms = 0;
    sbus->last_frame_ms = 0;
    sbus->failsafe = 1;

    for (int i = 0; i < 16; i++) {
        sbus->channels[i] = SBUS_CH_CENTER;
    }

    HAL_UART_Receive_IT(sbus->huart, &sbus->rx_byte, 1);
}

void SBUS_UART_RxCpltCallback(SBUS_t *sbus, UART_HandleTypeDef *huart)
{
    if (huart->Instance != sbus->huart->Instance) {
        return;
    }

    uint32_t now = HAL_GetTick();

    // Jeda antar paket SBUS > 3 ms menandakan awal frame baru
    if (now - sbus->last_byte_ms > 3) {
        sbus->idx = 0;
    }
    sbus->last_byte_ms = now;

    if (sbus->idx == 0 && sbus->rx_byte != SBUS_HEADER) {
        HAL_UART_Receive_IT(sbus->huart, &sbus->rx_byte, 1);
        return;
    }

    sbus->frame[sbus->idx++] = sbus->rx_byte;

    if (sbus->idx >= SBUS_FRAME_LEN) {
        if (sbus->frame[0] == SBUS_HEADER && sbus->frame[24] == SBUS_FOOTER) {
            SBUS_ParseFrame(sbus);
        }
        sbus->idx = 0;
    }

    HAL_UART_Receive_IT(sbus->huart, &sbus->rx_byte, 1);
}

uint8_t SBUS_IsConnected(SBUS_t *sbus)
{
    if (sbus->failsafe) return 0;
    if (HAL_GetTick() - sbus->last_frame_ms > 150) return 0; // Timeout > 150 ms
    return 1;
}

float SBUS_ChannelToFloat(uint16_t raw, float out_min, float out_max)
{
    if (raw < SBUS_CH_MIN) raw = SBUS_CH_MIN;
    if (raw > SBUS_CH_MAX) raw = SBUS_CH_MAX;
    return out_min + ((float)(raw - SBUS_CH_MIN) / (float)(SBUS_CH_MAX - SBUS_CH_MIN)) * (out_max - out_min);
}
