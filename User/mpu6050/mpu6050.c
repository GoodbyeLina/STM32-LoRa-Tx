#include "mpu6050.h"
#include <math.h>
#include <stdio.h>

#define RAD_TO_DEG 57.295779513082320876798154814105

// 定义滤波系数 (0.96 ~ 0.99 之间)
#define ALPHA 0.98f
#define DT 0.01f  // 假设 100Hz 的采样周期

uint8_t MPU6050_Init(I2C_HandleTypeDef *hi2c) {
		uint8_t check, data;
    HAL_StatusTypeDef status;

    // 尝试读取 WHO_AM_I 寄存器
    status = HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_WHO_AM_I, 1, &check, 1, 100);
    
    if (status != HAL_OK) {
        // I2C 通讯本身失败（可能是接线松动或没加上拉电阻）
        return 2; 
    }
		
    if (check == 0x68) {
        // 1. 唤醒传感器，内部 8MHz 时钟
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 1, &data, 1, 100);
        // 2. 设置采样率分频 = 7 (1kHz / (1+7) = 125Hz)
        data = 0x07;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_SMPRT_DIV, 1, &data, 1, 100);
        // 3. 配置加速度计量程 +/- 2g
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, 1, &data, 1, 100);
        // 4. 配置陀螺仪量程 +/- 250 deg/s
        data = 0x00;
        HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG, 1, &data, 1, 100);
        
        return 0; // 成功
    }
    return 1; // 失败
}

uint8_t MPU6050_Read_All(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *DataStruct) {
    uint8_t rec[14];
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, 1, rec, 14, 100) != HAL_OK) {
        return 1;
    }

    // 原始数据转换 (大端转小端)
    int16_t ax = (int16_t)(rec[0] << 8 | rec[1]);
    int16_t ay = (int16_t)(rec[2] << 8 | rec[3]);
    int16_t az = (int16_t)(rec[4] << 8 | rec[5]);
    int16_t temp = (int16_t)(rec[6] << 8 | rec[7]);
    int16_t gx = (int16_t)(rec[8] << 8 | rec[9]);
    int16_t gy = (int16_t)(rec[10] << 8 | rec[11]);
    int16_t gz = (int16_t)(rec[12] << 8 | rec[13]);

    // 转换为物理单位
    DataStruct->Accel_X = ax / 16384.0f;
    DataStruct->Accel_Y = ay / 16384.0f;
    DataStruct->Accel_Z = az / 16384.0f;
    DataStruct->Temperature = (float)((int16_t)temp / 340.0 + 36.53);
    DataStruct->Gyro_X = gx / 131.0f;
    DataStruct->Gyro_Y = gy / 131.0f;
    DataStruct->Gyro_Z = gz / 131.0f;

    // 计算姿态角 (加速度计方案)
    DataStruct->Pitch = atan2(DataStruct->Accel_Y, DataStruct->Accel_Z) * RAD_TO_DEG;
    DataStruct->Roll = atan2(-DataStruct->Accel_X, 
                       sqrt(DataStruct->Accel_Y * DataStruct->Accel_Y + 
                            DataStruct->Accel_Z * DataStruct->Accel_Z)) * RAD_TO_DEG;
    
		
    return 0;
}

void MPU6050_Read_Filtered(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *DataStruct) {
    uint8_t rec[14];
    HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, 0x3B, 1, rec, 14, 100);

    // 1. 转换原始数据
    int16_t ax = (int16_t)(rec[0] << 8 | rec[1]);
    int16_t ay = (int16_t)(rec[2] << 8 | rec[3]);
    int16_t az = (int16_t)(rec[4] << 8 | rec[5]);
    int16_t gx = (int16_t)(rec[8] << 8 | rec[9]);
    int16_t gy = (int16_t)(rec[10] << 8 | rec[11]);

    // 2. 转换为物理单位
    float acc_x = ax / 16384.0f;
    float acc_y = ay / 16384.0f;
    float acc_z = az / 16384.0f;
    float gyro_x = gx / 131.0f; 
    float gyro_y = gy / 131.0f;

    // 3. 计算加速度计角度 (静态稳定)
    float accel_pitch = atan2(acc_y, acc_z) * 57.295f;
    float accel_roll = atan2(-acc_x, sqrt(acc_y * acc_y + acc_z * acc_z)) * 57.295f;

    // 4. 互补滤波公式
    // 角度 = ALPHA * (当前角度 + 陀螺仪积分) + (1 - ALPHA) * 加速度计角度
    DataStruct->Pitch = ALPHA * (DataStruct->Pitch + gyro_x * DT) + (1.0f - ALPHA) * accel_pitch;
    DataStruct->Roll  = ALPHA * (DataStruct->Roll + gyro_y * DT) + (1.0f - ALPHA) * accel_roll;
}

void MPU6050_Calibrate(I2C_HandleTypeDef *hi2c, MPU6050_Data_t *DataStruct) {
    float gx_sum = 0, gy_sum = 0;
    for(int i = 0; i < 100; i++) {
        MPU6050_Read_All(hi2c, DataStruct);
        gx_sum += DataStruct->Gyro_X;
        gy_sum += DataStruct->Gyro_Y;
        HAL_Delay(5);
    }
    DataStruct->Gyro_X_Offset = gx_sum / 100.0f;
    DataStruct->Gyro_Y_Offset = gy_sum / 100.0f;
}

/**
 * @brief 根据选择模式打印数据
 * @param DataStruct 指向数据的结构体
 * @param Mode 模式选择：MPU6050_MODE_ACCEL 或 MPU6050_MODE_ANGLES
 */
void MPU6050_PrintData(MPU6050_Data_t *DataStruct, MPU6050_PrintMode_t Mode) {
    if (Mode == MPU6050_MODE_ACCEL) {
        // 打印原始加速度数据 (单位: g)
        printf("Accel RAW -> X:%.2f g, Y:%.2f g, Z:%.2f g\r\n", 
                DataStruct->Accel_X, 
                DataStruct->Accel_Y, 
                DataStruct->Accel_Z);
    } 
    else if (Mode == MPU6050_MODE_ANGLES) {
        // 打印互补滤波后的姿态角 (单位: °)
        printf("Attitude -> Pitch:%.2f°, Roll:%.2f°\r\n", 
                DataStruct->Pitch, 
                DataStruct->Roll);
    }
}
