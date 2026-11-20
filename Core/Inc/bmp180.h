#ifndef BMP180_H
#define BMP180_H

#include "main.h"

#define BMP180_I2C_ADDR  (0x77 << 1)
#define BMP180_OSS       1  // Oversampling standard (konversi ~7.5 ms)

typedef struct {
    // Parameter kalibrasi pabrik (Big-Endian, register 0xAA..0xBF)
    int16_t  AC1, AC2, AC3;
    uint16_t AC4, AC5, AC6;
    int16_t  B1, B2, MB, MC, MD;

    // State machine pembacaan non-blocking
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
} BMP180_t;

uint8_t BMP180_Init(I2C_HandleTypeDef *hi2c, BMP180_t *dev);
void    BMP180_Update(I2C_HandleTypeDef *hi2c, BMP180_t *dev);
void    BMP180_Calibrate_Altitude(I2C_HandleTypeDef *hi2c, BMP180_t *dev, uint16_t samples);

#endif /* BMP180_H */
