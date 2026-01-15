#ifndef __LORA_H
#define __LORA_H

#include "main.h"

// 模式定义
typedef enum {
		LORA_MODE_NORMAL   = 0, // M0=0, M1=0 高时效模式（透传）
    LORA_MODE_WAKEUP   = 1, // M0=1, M1=0 空中唤醒模式
    LORA_MODE_AT       = 2, // M0=0, M1=1 AT 模式 (重点修改)
    LORA_MODE_SLEEP    = 3  // M0=1, M1=1 休眠模式
} LoRa_Mode_t;

// 建议放在 lora.h 中
#pragma pack(1) // 必须强制 1 字节对齐
typedef struct {
    // VOFA+ JustFloat 不需要帧头，它靠帧尾识别
    float temperature;
    float humidity;
    float acc_x;
    float acc_y;
    float acc_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float latitude;
    float longitude;
    // VOFA+ 专用帧尾：0x00 0x00 0x80 0x7F
    uint32_t tail; 
} LoRa_Packet_t;
#pragma pack()

// 引脚操作简写
#define LORA_AUX_STATE  HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9)

void LoRa_Init(void);
void LoRa_SetMode(LoRa_Mode_t mode);
HAL_StatusTypeDef LoRa_Send(uint8_t *pData, uint16_t Size);

#endif
