#include "aht20.h"
#include <math.h>

uint8_t AHT20_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t data;
    HAL_Delay(40); // 等待上电稳定
    HAL_I2C_Mem_Read(hi2c, AHT20_ADDRESS, 0x71, 1, &data, 1, 100);
    if ((data & 0x08) == 0) { // 检查校准使能位
        uint8_t init_cmd[] = {0xBE, 0x08, 0x00};
        HAL_I2C_Master_Transmit(hi2c, AHT20_ADDRESS, init_cmd, 3, 100);
        return 1;
    }
    return 0;
}

uint8_t AHT20_Read(I2C_HandleTypeDef *hi2c, AHT20_Data_t *DataStruct) {
    uint8_t cmd[] = {0xAC, 0x33, 0x00};
    uint8_t data[6];

    HAL_I2C_Master_Transmit(hi2c, AHT20_ADDRESS, cmd, 3, 100);
    HAL_Delay(80); // 必须等待转换完成

    if (HAL_I2C_Master_Receive(hi2c, AHT20_ADDRESS, data, 6, 100) == HAL_OK) {
        // 湿度计算
        uint32_t hum_raw = ((uint32_t)data[1] << 12) | ((uint32_t)data[2] << 4) | (data[3] >> 4);
        DataStruct->Humidity = hum_raw * 100.0f / 1048576.0f;
        // 温度计算
        uint32_t temp_raw = (((uint32_t)data[3] & 0x0F) << 16) | ((uint32_t)data[4] << 8) | data[5];
        DataStruct->Temperature = temp_raw * 200.0f / 1048576.0f - 50.0f;
        return 0;
    }
    return 1;
}
