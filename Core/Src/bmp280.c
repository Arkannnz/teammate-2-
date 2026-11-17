#include "bmp280.h"
#include <math.h>

static void BMP280_Read_Raw(I2C_HandleTypeDef *hi2c, BMP280_t *dev, int32_t *adc_P, int32_t *adc_T)
{
    uint8_t data[6] = {0};
    HAL_I2C_Mem_Read(hi2c, dev->dev_addr, 0xF7, 1, data, 6, 100);

    *adc_P = ((int32_t)data[0] << 12) | ((int32_t)data[1] << 4) | ((int32_t)data[2] >> 4);
    *adc_T = ((int32_t)data[3] << 12) | ((int32_t)data[4] << 4) | ((int32_t)data[5] >> 4);
}

uint8_t BMP280_Init(I2C_HandleTypeDef *hi2c, BMP280_t *dev)
{
    uint8_t chip_id = 0;

    // Deteksi otomatis alamat I2C BMP280/BME280 (0x76 atau 0x77)
    if (HAL_I2C_IsDeviceReady(hi2c, (0x76 << 1), 2, 50) == HAL_OK) {
        dev->dev_addr = (0x76 << 1);
    } else if (HAL_I2C_IsDeviceReady(hi2c, (0x77 << 1), 2, 50) == HAL_OK) {
        dev->dev_addr = (0x77 << 1);
    } else {
        return 0; // Sensor tidak ditemukan
    }

    HAL_I2C_Mem_Read(hi2c, dev->dev_addr, 0xD0, 1, &chip_id, 1, 100);
    if (chip_id != 0x58 && chip_id != 0x60) {
        return 0;
    }

    // Baca 24 byte parameter kalibrasi pabrik
    uint8_t calib[24];
    HAL_I2C_Mem_Read(hi2c, dev->dev_addr, 0x88, 1, calib, 24, 100);

    dev->dig_T1 = (uint16_t)(((uint16_t)calib[1] << 8) | calib[0]);
    dev->dig_T2 = (int16_t)(((uint16_t)calib[3] << 8) | calib[2]);
    dev->dig_T3 = (int16_t)(((uint16_t)calib[5] << 8) | calib[4]);

    dev->dig_P1 = (uint16_t)(((uint16_t)calib[7] << 8) | calib[6]);
    dev->dig_P2 = (int16_t)(((uint16_t)calib[9] << 8) | calib[8]);
    dev->dig_P3 = (int16_t)(((uint16_t)calib[11] << 8) | calib[10]);
    dev->dig_P4 = (int16_t)(((uint16_t)calib[13] << 8) | calib[12]);
    dev->dig_P5 = (int16_t)(((uint16_t)calib[15] << 8) | calib[14]);
    dev->dig_P6 = (int16_t)(((uint16_t)calib[17] << 8) | calib[16]);
    dev->dig_P7 = (int16_t)(((uint16_t)calib[19] << 8) | calib[18]);
    dev->dig_P8 = (int16_t)(((uint16_t)calib[21] << 8) | calib[20]);
    dev->dig_P9 = (int16_t)(((uint16_t)calib[23] << 8) | calib[22]);

    // Konfigurasi: Normal mode, Oversampling Temp x2, Press x16, Filter IIR x4
    uint8_t config = 0x08;
    uint8_t ctrl_meas = 0x57;
    HAL_I2C_Mem_Write(hi2c, dev->dev_addr, 0xF5, 1, &config, 1, 100);
    HAL_I2C_Mem_Write(hi2c, dev->dev_addr, 0xF4, 1, &ctrl_meas, 1, 100);

    dev->base_pressure = 1013.25f;
    dev->temperature = 0.0f;
    dev->pressure = 1013.25f;
    dev->altitude = 0.0f;
    return 1;
}

void BMP280_Read_All(I2C_HandleTypeDef *hi2c, BMP280_t *dev)
{
    int32_t adc_P, adc_T;
    BMP280_Read_Raw(hi2c, dev, &adc_P, &adc_T);

    // Kompensasi Suhu (rumus resmi Bosch)
    int32_t var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
    int32_t var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >> 12) * ((int32_t)dev->dig_T3)) >> 14;
    int32_t t_fine = var1 + var2;
    dev->temperature = ((t_fine * 5 + 128) >> 8) / 100.0f;

    // Kompensasi Tekanan
    int64_t p_var1 = ((int64_t)t_fine) - 128000;
    int64_t p_var2 = p_var1 * p_var1 * (int64_t)dev->dig_P6;
    p_var2 = p_var2 + ((p_var1 * (int64_t)dev->dig_P5) << 17);
    p_var2 = p_var2 + (((int64_t)dev->dig_P4) << 35);
    p_var1 = ((p_var1 * p_var1 * (int64_t)dev->dig_P3) >> 8) + ((p_var1 * (int64_t)dev->dig_P2) << 12);
    p_var1 = (((((int64_t)1) << 47) + p_var1)) * ((int64_t)dev->dig_P1) >> 33;

    if (p_var1 == 0) {
        return;
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - p_var2) * 3125) / p_var1;
    p_var1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    p_var2 = (((int64_t)dev->dig_P8) * p) >> 19;
    p = ((p + p_var1 + p_var2) >> 8) + (((int64_t)dev->dig_P7) << 4);

    dev->pressure = (float)p / 25600.0f; // dalam hPa
    dev->altitude = 44330.0f * (1.0f - powf(dev->pressure / dev->base_pressure, 0.19029495f));
}

void BMP280_Calibrate_Altitude(I2C_HandleTypeDef *hi2c, BMP280_t *dev, uint16_t samples)
{
    float sum = 0.0f;
    for (uint16_t i = 0; i < samples; i++) {
        BMP280_Read_All(hi2c, dev);
        sum += dev->pressure;
        HAL_Delay(10);
    }
    if (samples > 0) {
        dev->base_pressure = sum / (float)samples;
        dev->altitude = 0.0f;
    }
}
