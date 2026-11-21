/*
 * bmp280.h
 *
 *  Created on: Nov 21, 2026
 *      Author: eanad
 */

#ifndef INC_BMP280_H_
#define INC_BMP280_H_

#ifndef BMP280_H
#define BMP280_H

#include "main.h"
#include <stdint.h>

#define BMP280_I2C_ADDR  (0x76 << 1)

typedef struct {
    // Parameter kalibrasi pabrik BMP280/BME280 (Little-Endian, register 0x88..0x9F)
    uint16_t dig_T1, dig_P1;
    int16_t  dig_T2, dig_T3;
    int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t  t_fine;

    // State machine pembacaan non-blocking (sama persis seperti bmp180.h)
    uint8_t  state;
    uint32_t state_tick;
    int32_t  raw_ut;
    int32_t  raw_up;
    uint8_t  data_ready;

    // Hasil pembacaan terkompensasi
    float    temperature;    // °C
    float    pressure;       // hPa
    float    altitude;       // meter (relatif terhadap posisi awal)
    float    base_pressure;  // hPa referensi saat kalibrasi
} BMP280_t;

uint8_t BMP280_Init(I2C_HandleTypeDef *hi2c, BMP280_t *dev);
void    BMP280_Update(I2C_HandleTypeDef *hi2c, BMP280_t *dev);
void    BMP280_Calibrate_Altitude(I2C_HandleTypeDef *hi2c, BMP280_t *dev, uint16_t samples);

#endif /* BMP280_H */


#endif /* INC_BMP280_H_ */
