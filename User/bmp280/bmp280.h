#ifndef __BMP280_H
#define __BMP280_H

#include "main.h"

#define BMP280_ADDR 0xEC // 0x76 << 1 (若引脚接高电平则为 0x77<<1)

typedef struct {
    float Temperature;
    float Pressure;
    float Altitude;
} BMP280_Data_t;

uint8_t BMP280_Init(I2C_HandleTypeDef *hi2c);
void BMP280_Read(I2C_HandleTypeDef *hi2c, BMP280_Data_t *DataStruct);

#endif
