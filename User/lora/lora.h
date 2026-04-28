#ifndef __LORA_H
#define __LORA_H

#include "main.h"
#include <stddef.h>  // offsetof

// 模式定义
typedef enum {
		LORA_MODE_NORMAL   = 0, // M0=0, M1=0 高时效模式（透传）
    LORA_MODE_WAKEUP   = 1, // M0=1, M1=0 空中唤醒模式
    LORA_MODE_AT       = 2, // M0=0, M1=1 AT 模式 (重点修改)
    LORA_MODE_SLEEP    = 3  // M0=1, M1=1 休眠模式
} LoRa_Mode_t;

// ========================================
// LoRa 数据包协议 (v2.0 - RTK 扩展版)
// 兼容 JustFloat 帧尾格式
// ========================================
#define LORA_PACKET_VER 0x02  // 协议版本号

#pragma pack(1) // 必须强制 1 字节对齐
typedef struct {
    // ========== 环境数据 (8 bytes) ==========
    float temperature;   // AHT20 温度 (°C)
    float humidity;      // AHT20 湿度 (%)
    
    // ========== 姿态数据 (24 bytes) ==========
    float acc_x;         // MPU6050 X轴加速度 (g)
    float acc_y;         // MPU6050 Y轴加速度 (g)
    float acc_z;         // MPU6050 Z轴加速度 (g)
    float gyro_x;        // MPU6050 X轴角速度 (°/s)
    float gyro_y;        // MPU6050 Y轴角速度 (°/s)
    float gyro_z;        // MPU6050 Z轴角速度 (°/s)
    
    // ========== 定位数据 (16 bytes) ==========
    float latitude;      // UM982 RTK 纬度 (十进制度)
    float longitude;     // UM982 RTK 经度 (十进制度)
    float altitude;      // UM982 RTK 海拔高度 (m)    ← 🆕
    float hdop;          // 水平精度因子              ← 🆕
    
    // ========== RTK 状态 (4 bytes) ==========
    uint8_t fix_type;    // RTK 解状态               ← 🆕
                         //   0=无效, 1=单点, 4=固定解, 5=浮点解
    uint8_t satellites;  // 跟踪卫星数                ← 🆕
    uint8_t reserved[2]; // 保留字节 (用于未来扩展)    ← 🆕
    
    // ========== 校验 (6 bytes) ==========
    uint16_t crc16;      // CRC16, 覆盖 crc16 之前的所有数据
    uint32_t tail;       // VOFA+ JustFloat 帧尾: 0x7F800000
} LoRa_Packet_t;
#pragma pack()

// 数据包大小常量
#define LORA_DATA_SIZE      offsetof(LoRa_Packet_t, crc16)  // CRC 校验范围
#define LORA_PACKET_SIZE    sizeof(LoRa_Packet_t)           // 总包大小

// 引脚操作简写
#define LORA_AUX_STATE  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)

void LoRa_Init(void);
void LoRa_SetMode(LoRa_Mode_t mode);
HAL_StatusTypeDef LoRa_Send(uint8_t *pData, uint16_t Size);

#endif
