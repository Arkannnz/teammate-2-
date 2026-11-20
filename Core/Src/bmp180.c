#include "bmp180.h"
#include <math.h>

static void BMP180_Compute(BMP180_t *dev)
{
    int32_t X1, X2, X3, B3, B5, B6, p;
    uint32_t B4, B7;

    // 1. Kompensasi Suhu (rumus resmi datasheet Bosch BMP180)
    X1 = ((dev->raw_ut - (int32_t)dev->AC6) * (int32_t)dev->AC5) >> 15;
    int32_t denom_t = X1 + (int32_t)dev->MD;
    if (denom_t == 0) return;
    X2 = ((int32_t)dev->MC << 11) / denom_t;
    B5 = X1 + X2;
    dev->temperature = (float)((B5 + 8) >> 4) / 10.0f;

    // 2. Kompensasi Tekanan
    B6 = B5 - 4000;
    X1 = ((int32_t)dev->B2 * ((B6 * B6) >> 12)) >> 11;
    X2 = ((int32_t)dev->AC2 * B6) >> 11;
    X3 = X1 + X2;
    B3 = (((((int32_t)dev->AC1) * 4 + X3) << BMP180_OSS) + 2) >> 2;

    X1 = ((int32_t)dev->AC3 * B6) >> 13;
    X2 = ((int32_t)dev->B1 * ((B6 * B6) >> 12)) >> 16;
    X3 = ((X1 + X2) + 2) >> 2;
    B4 = ((uint32_t)dev->AC4 * (uint32_t)(X3 + 32768)) >> 15;
    if (B4 == 0) return;

    B7 = ((uint32_t)dev->raw_up - (uint32_t)B3) * (uint32_t)(50000UL >> BMP180_OSS);
    if (B7 < 0x80000000UL) {
        p = (int32_t)((B7 * 2UL) / B4);
    } else {
        p = (int32_t)((B7 / B4) * 2UL);
    }

    X1 = (p >> 8) * (p >> 8);
    X1 = (X1 * 3038) >> 16;
    X2 = (-7357 * p) >> 16;
    p  = p + ((X1 + X2 + 3791) >> 4);

    dev->pressure = (float)p / 100.0f; // Pa -> hPa
    dev->altitude = 44330.0f * (1.0f - powf(dev->pressure / dev->base_pressure, 0.19029495f));
    dev->data_ready = 1;
}

uint8_t BMP180_Init(I2C_HandleTypeDef *hi2c, BMP180_t *dev)
{
    uint8_t chip_id = 0;

    if (HAL_I2C_IsDeviceReady(hi2c, BMP180_I2C_ADDR, 3, 50) != HAL_OK) {
        return 0;
    }

    // Chip ID BMP180 wajib 0x55
    if (HAL_I2C_Mem_Read(hi2c, BMP180_I2C_ADDR, 0xD0, 1, &chip_id, 1, 100) != HAL_OK) {
        return 0;
    }
    if (chip_id != 0x55) {
        return 0;
    }

    // Baca 22 byte koefisien kalibrasi (0xAA..0xBF, Big-Endian)
    uint8_t calib[22] = {0};
    if (HAL_I2C_Mem_Read(hi2c, BMP180_I2C_ADDR, 0xAA, 1, calib, 22, 100) != HAL_OK) {
        return 0;
    }

    dev->AC1 = (int16_t)((calib[0]  << 8) | calib[1]);
    dev->AC2 = (int16_t)((calib[2]  << 8) | calib[3]);
    dev->AC3 = (int16_t)((calib[4]  << 8) | calib[5]);
    dev->AC4 = (uint16_t)((calib[6] << 8) | calib[7]);
    dev->AC5 = (uint16_t)((calib[8] << 8) | calib[9]);
    dev->AC6 = (uint16_t)((calib[10]<< 8) | calib[11]);
    dev->B1  = (int16_t)((calib[12] << 8) | calib[13]);
    dev->B2  = (int16_t)((calib[14] << 8) | calib[15]);
    dev->MB  = (int16_t)((calib[16] << 8) | calib[17]);
    dev->MC  = (int16_t)((calib[18] << 8) | calib[19]);
    dev->MD  = (int16_t)((calib[20] << 8) | calib[21]);

    dev->base_pressure = 1013.25f;
    dev->temperature   = 0.0f;
    dev->pressure      = 1013.25f;
    dev->altitude      = 0.0f;
    dev->state         = 0;
    dev->data_ready    = 0;

    return 1;
}

void BMP180_Update(I2C_HandleTypeDef *hi2c, BMP180_t *dev)
{
    uint32_t now = HAL_GetTick();

    if (dev->state == 0) {
        // Picu pengukuran suhu
        uint8_t cmd = 0x2E;
        if (HAL_I2C_Mem_Write(hi2c, BMP180_I2C_ADDR, 0xF4, 1, &cmd, 1, 10) == HAL_OK) {
            dev->state_tick = now;
            dev->state = 1;
        }
    } else if (dev->state == 1) {
        // Tunggu 5 ms lalu baca suhu & picu pengukuran tekanan
        if ((now - dev->state_tick) >= 5U) {
            uint8_t raw_t[2] = {0};
            if (HAL_I2C_Mem_Read(hi2c, BMP180_I2C_ADDR, 0xF6, 1, raw_t, 2, 10) == HAL_OK) {
                dev->raw_ut = (int32_t)(((uint16_t)raw_t[0] << 8) | raw_t[1]);
                uint8_t cmd = (uint8_t)(0x34 + (BMP180_OSS << 6));
                if (HAL_I2C_Mem_Write(hi2c, BMP180_I2C_ADDR, 0xF4, 1, &cmd, 1, 10) == HAL_OK) {
                    dev->state_tick = now;
                    dev->state = 2;
                }
            }
        }
    } else if (dev->state == 2) {
        // Tunggu 8 ms lalu baca tekanan, hitung altitude, & picu suhu berikutnya
        if ((now - dev->state_tick) >= 8U) {
            uint8_t raw_p[3] = {0};
            if (HAL_I2C_Mem_Read(hi2c, BMP180_I2C_ADDR, 0xF6, 1, raw_p, 3, 10) == HAL_OK) {
                dev->raw_up = (int32_t)((((uint32_t)raw_p[0] << 16) |
                                         ((uint32_t)raw_p[1] << 8)  |
                                          (uint32_t)raw_p[2]) >> (8 - BMP180_OSS));
                BMP180_Compute(dev);
            }
            dev->state = 0;
        }
    }
}

void BMP180_Calibrate_Altitude(I2C_HandleTypeDef *hi2c, BMP180_t *dev, uint16_t samples)
{
    float sum = 0.0f;
    uint16_t count = 0;
    uint32_t timeout = HAL_GetTick() + (samples * 25U) + 200U;

    while (count < samples && HAL_GetTick() < timeout) {
        dev->data_ready = 0;
        BMP180_Update(hi2c, dev);
        if (dev->data_ready) {
            sum += dev->pressure;
            count++;
        }
        HAL_Delay(1);
    }

    if (count > 0) {
        dev->base_pressure = sum / (float)count;
        dev->altitude = 0.0f;
    }
}
