#ifndef BMP280_H
#define BMP280_H

#include "main.h"

typedef struct {
    uint8_t  dev_addr;
    uint16_t dig_T1;
    int16_t  dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    float temperature;    // °C
    float pressure;       // hPa
    float altitude;       // meter (relatif terhadap posisi awal)
    float base_pressure;  // tekanan referensi saat kalibrasi awal (hPa)
} BMP280_t;

uint8_t BMP280_Init(I2C_HandleTypeDef *hi2c, BMP280_t *dev);
void    BMP280_Calibrate_Altitude(I2C_HandleTypeDef *hi2c, BMP280_t *dev, uint16_t samples);
void    BMP280_Read_All(I2C_HandleTypeDef *hi2c, BMP280_t *dev);

#endif /* BMP280_H */
