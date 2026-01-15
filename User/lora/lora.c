#include "lora.h"
#include "usart.h"

extern UART_HandleTypeDef huart3;


/**
 * @brief 发送数据
 */
HAL_StatusTypeDef LoRa_Send(uint8_t *pData, uint16_t Size) {
    HAL_Delay(100);
    return HAL_UART_Transmit(&huart3, pData, Size, 1000);
}

