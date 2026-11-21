/*
 * bmp280.c
 *
 *  Created on: Nov 21, 2026
 *      Author: eanad
 */

#include "bmp280.h"
#include <math.h>

static void BMP280_Compute(BMP280_t *dev)
{
    // 1. Kompensasi Suhu (rumus resmi datasheet Bosch BMP280)
    int32_t tv1 = ((((dev->raw_ut >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
    int32_t tv2 = (((((dev->raw_ut >> 4) - ((int32_t)dev->dig_T1)) *
                     ((dev->raw_ut >> 4) - ((int32_t)dev->dig_T1))) >> 12) * ((int32_t)dev->dig_T3)) >> 14;
    dev->t_fine = tv1 + tv2;
    dev->temperature = (float)((dev->t_fine * 5 + 128) >> 8) / 100.0f;

    // 2. Kompensasi Tekanan (rumus resmi datasheet Bosch BMP280)
    int64_t var1 = ((int64_t)dev->t_fine) - 128000;
    int64_t var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->dig_P1) >> 33;
    if (var1 == 0) return;

    int64_t p = 1048576 - dev->raw_up;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);

    dev->pressure = ((float)p / 256.0f) / 100.0f; // Pa -> hPa
    if (dev->base_pressure > 0.0f && dev->pressure > 0.0f) {
        dev->altitude = 44330.0f * (1.0f - powf(dev->pressure / dev->base_pressure, 0.19029495f));
    }
    dev->data_ready = 1;
}

uint8_t BMP280_Init(I2C_HandleTypeDef *hi2c, BMP280_t *dev)
{
    uint8_t chip_id = 0;

    if (HAL_I2C_IsDeviceReady(hi2c, BMP280_I2C_ADDR, 3, 50) != HAL_OK) {
        return 0;
    }

    // Validasi Chip ID: 0x58 (BMP280) atau 0x60 (BME280)
    if (HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDR, 0xD0, 1, &chip_id, 1, 100) != HAL_OK) {
        return 0;
    }
    if (chip_id != 0x58 && chip_id != 0x60) {
        return 0;
    }

    // Soft reset
    uint8_t reset_cmd = 0xB6;
    HAL_I2C_Mem_Write(hi2c, BMP280_I2C_ADDR, 0xE0, 1, &reset_cmd, 1, 100);
    HAL_Delay(10);

    // Baca 24 byte koefisien kalibrasi pabrik (0x88..0x9F, Little-Endian)
    uint8_t calib[24] = {0};
    if (HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDR, 0x88, 1, calib, 24, 100) != HAL_OK) {
        return 0;
    }

    dev->dig_T1 = (uint16_t)((calib[1]  << 8) | calib[0]);
    dev->dig_T2 = (int16_t) ((calib[3]  << 8) | calib[2]);
    dev->dig_T3 = (int16_t) ((calib[5]  << 8) | calib[4]);
    dev->dig_P1 = (uint16_t)((calib[7]  << 8) | calib[6]);
    dev->dig_P2 = (int16_t) ((calib[9]  << 8) | calib[8]);
    dev->dig_P3 = (int16_t) ((calib[11] << 8) | calib[10]);
    dev->dig_P4 = (int16_t) ((calib[13] << 8) | calib[12]);
    dev->dig_P5 = (int16_t) ((calib[15] << 8) | calib[14]);
    dev->dig_P6 = (int16_t) ((calib[17] << 8) | calib[16]);
    dev->dig_P7 = (int16_t) ((calib[19] << 8) | calib[18]);
    dev->dig_P8 = (int16_t) ((calib[21] << 8) | calib[20]);
    dev->dig_P9 = (int16_t) ((calib[23] << 8) | calib[22]);

    // IIR Filter = 16 (0xF5), Standby = 0.5 ms
    uint8_t cfg  = (0x00 << 5) | (0x04 << 2);
    // Normal Mode: osrs_t = x2, osrs_p = x16 (0xF4)
    uint8_t ctrl = (0x02 << 5) | (0x05 << 2) | 0x03;
    if (HAL_I2C_Mem_Write(hi2c, BMP280_I2C_ADDR, 0xF5, 1, &cfg, 1, 100) != HAL_OK) return 0;
    if (HAL_I2C_Mem_Write(hi2c, BMP280_I2C_ADDR, 0xF4, 1, &ctrl, 1, 100) != HAL_OK) return 0;

    dev->base_pressure = 1013.25f;
    dev->temperature   = 0.0f;
    dev->pressure      = 1013.25f;
    dev->altitude      = 0.0f;
    dev->state         = 0;
    dev->state_tick    = 0;
    dev->data_ready    = 0;

    HAL_Delay(20);
    return 1;
}

void BMP280_Update(I2C_HandleTypeDef *hi2c, BMP280_t *dev)
{
    uint32_t now = HAL_GetTick();

    // Non-blocking throttle setiap 40 ms (menyesuaikan rate konversi x16 oversampling)
    if (dev->state == 1 && (now - dev->state_tick) < 40U) {
        return;
    }

    uint8_t raw[6] = {0};
    if (HAL_I2C_Mem_Read(hi2c, BMP280_I2C_ADDR, 0xF7, 1, raw, 6, 10) == HAL_OK) {
        dev->raw_up = (int32_t)(((uint32_t)raw[0] << 12) | ((uint32_t)raw[1] << 4) | ((uint32_t)raw[2] >> 4));
        dev->raw_ut = (int32_t)(((uint32_t)raw[3] << 12) | ((uint32_t)raw[4] << 4) | ((uint32_t)raw[5] >> 4));
        BMP280_Compute(dev);
        dev->state_tick = now;
        dev->state = 1;
    }
}

void BMP280_Calibrate_Altitude(I2C_HandleTypeDef *hi2c, BMP280_t *dev, uint16_t samples)
{
    float sum = 0.0f;
    uint16_t count = 0;
    uint32_t timeout = HAL_GetTick() + (samples * 45U) + 200U;

    while (count < samples && HAL_GetTick() < timeout) {
        dev->data_ready = 0;
        BMP280_Update(hi2c, dev);
        if (dev->data_ready && dev->pressure > 0.0f) {
            sum += dev->pressure;
            count++;
        }
        HAL_Delay(5);
    }

    if (count > 0) {
        dev->base_pressure = sum / (float)count;
        dev->altitude = 0.0f;
    }
}
