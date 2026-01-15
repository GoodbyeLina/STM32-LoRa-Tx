#ifndef __AHT20_H
#define __AHT20_H

#include "main.h"

#define AHT20_ADDRESS 0x70 // 0x38 << 1

typedef struct {
    float Temperature;
    float Humidity;
} AHT20_Data_t;

uint8_t AHT20_Init(I2C_HandleTypeDef *hi2c);
uint8_t AHT20_Read(I2C_HandleTypeDef *hi2c, AHT20_Data_t *DataStruct);

#endif
