#include "bmp280.h"
#include <math.h>

// 校准参数结构体
struct {
    uint16_t dig_T1; int16_t dig_T2, dig_T3;
    uint16_t dig_P1; int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;
    int32_t t_fine;
} calib;

uint8_t BMP280_Init(I2C_HandleTypeDef *hi2c) {
    uint8_t id, params[24];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, 0xD0, 1, &id, 1, 100);
		printf("BMP280 ID: 0x%02X\n", id); // 看看打印出来的是 0x60, 0x00 还是 0xFF
//    if (id != 0x58) return 1;

    // 1. 读取补偿参数
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, 0x88, 1, params, 24, 100);
    calib.dig_T1 = (params[1]<<8)|params[0]; 
		calib.dig_T2 = (params[3]<<8)|params[2]; // ...以此类推
		calib.dig_T3 = (params[5]<<8)|params[4]; // ...以此类推
		calib.dig_P1 = (params[7]<<8)|params[6]; // ...以此类推
		calib.dig_P2 = (params[9]<<8)|params[8]; // ...以此类推
		calib.dig_P3 = (params[11]<<8)|params[10]; // ...以此类推
		calib.dig_P4 = (params[13]<<8)|params[12]; // ...以此类推
		calib.dig_P5 = (params[15]<<8)|params[14]; // ...以此类推
		calib.dig_P6 = (params[17]<<8)|params[16]; // ...以此类推
		calib.dig_P7 = (params[19]<<8)|params[18]; // ...以此类推
		calib.dig_P8 = (params[21]<<8)|params[20]; // ...以此类推
		calib.dig_P9 = (params[23]<<8)|params[22]; // ...以此类推
    // (简略写法：实际需手动补全所有 P1-P9 的赋值)

    // 2. 配置传感器 (Ctrl_meas: 2倍过采样, 正常模式)
    uint8_t config = 0x57; 
    HAL_I2C_Mem_Write(hi2c, BMP280_ADDR, 0xF4, 1, &config, 1, 100);
    return 0;
}

// 补偿算法 (简化浮点版)
void BMP280_Read(I2C_HandleTypeDef *hi2c, BMP280_Data_t *DataStruct) {
    uint8_t data[6];
    HAL_I2C_Mem_Read(hi2c, BMP280_ADDR, 0xF7, 1, data, 6, 100);
    int32_t adc_P = (data[0]<<12)|(data[1]<<4)|(data[2]>>4);
    int32_t adc_T = (data[3]<<12)|(data[4]<<4)|(data[5]>>4);

    // 温度补偿 (计算 t_fine)
    float v1 = (((float)adc_T)/16384.0f - ((float)calib.dig_T1)/1024.0f) * ((float)calib.dig_T2);
    float v2 = ((((float)adc_T)/131072.0f - ((float)calib.dig_T1)/8192.0f) * (((float)adc_T)/131072.0f - ((float)calib.dig_T1)/8192.0f)) * ((float)calib.dig_T3);
    DataStruct->Temperature = (v1 + v2) / 5120.0f;
    
    // 气压补偿逻辑类似 (此处省略冗长公式)
    // DataStruct->Pressure = ...;
    
    // 计算高度 (海平面标准大气压 1013.25hPa)
    DataStruct->Altitude = 44330.0f * (1.0f - pow(DataStruct->Pressure / 101325.0f, 0.1903f));
}
