#ifndef __GPS_H
#define __GPS_H

#include "main.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"

// 增大缓冲区以容纳完整的 NMEA 语句（建议 256）
#define GPS_BUF_SIZE 256

typedef struct {
    uint8_t is_valid;   // 是否定位成功
    float latitude;     // 纬度 (十进制度)
    float longitude;    // 经度 (十进制度)
} GPS_Data_t;

extern GPS_Data_t g_gpsData; // 声明一个全局变量供 main 使用

// 外部声明，方便 main.c 使用标志位
extern char gps_line_buffer[GPS_BUF_SIZE];
extern uint8_t gps_line_ready;
extern uint8_t gps_rx_byte;

// 函数声明
uint8_t GPS_Init(void);
void GPS_Process(void);
void GPS_Callback(UART_HandleTypeDef *huart);
float NMEA_To_Degree(char *str);


#endif
