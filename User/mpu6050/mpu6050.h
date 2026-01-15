#ifndef __MPU6050_H
#define __MPU6050_H

#include "main.h"

// 默认 I2C 地址 (AD0 接地为 0x68, 左移一位得到 0xD0)
#define MPU6050_ADDR         (0x68 << 1)

// 常用寄存器地址
#define MPU6050_SMPRT_DIV    0x19
#define MPU6050_CONFIG       0x1A
#define MPU6050_GYRO_CONFIG  0x1B
#define MPU6050_ACCEL_CONFIG 0x1C
#define MPU6050_WHO_AM_I     0x75
#define MPU6050_PWR_MGMT_1   0x6B
#define MPU6050_ACCEL_XOUT_H 0x3B

typedef struct {
    float Accel_X, Accel_Y, Accel_Z;
    float Gyro_X, Gyro_Y, Gyro_Z;
    float Temperature;
    float Pitch, Roll; 
		float Gyro_X_Offset, Gyro_Y_Offset; // <--- 必须添加这两行
} MPU6050_Data_t;

// 定义输出模式枚举
typedef enum {
    MPU6050_MODE_ACCEL,  // 仅打印加速度 (X, Y, Z)
    MPU6050_MODE_ANGLES  // 仅打印处理后的偏转角 (Pitch, Roll)
} MPU6050_PrintMode_t;

// 传入 I2C 句柄，增加驱动的复用性
uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c);
uint8_t MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *DataStruct);
void MPU6050_Read_Filtered(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *DataStruct);
void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *DataStruct);
// 新增打印控制函数
void MPU6050_PrintData(MPU6050_Data_t *DataStruct, MPU6050_PrintMode_t Mode);

#endif
