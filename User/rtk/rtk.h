#ifndef __RTK_H
#define __RTK_H

#include "main.h"
#include "usart.h"
#include "string.h"
#include "stdio.h"
#include <stdlib.h>

/* ========================================
 * UM982 RTK 模块驱动 (v1.0)
 * 接口: USART2 (PA2-TX, PA3-RX)
 * 波特率: 115200
 * 协议: NMEA-0183 (GGA, GSA, RMC)
 * ======================================== */

// NMEA 行缓冲区大小
#define RTK_BUF_SIZE        256

// RTK 解状态枚举
typedef enum {
    RTK_FIX_INVALID     = 0,  // 无效
    RTK_FIX_SINGLE      = 1,  // 单点定位 (SPS)
    RTK_FIX_DGPS        = 2,  // 伪距差分
    RTK_FIX_PPS         = 3,  // 精密定位
    RTK_FIX_FIXED       = 4,  // ⭐ RTK 固定解 (厘米级)
    RTK_FIX_FLOAT       = 5,  // ⭐ RTK 浮点解 (分米级)
    RTK_FIX_DEAD_RECK   = 6,  // 航位推算
    RTK_FIX_MANUAL      = 7,  // 手动输入
    RTK_FIX_SIMULATION  = 8   // 仿真
} RTK_FixType_t;

// RTK 数据结构体
typedef struct {
    uint8_t     is_valid;       // 是否有效定位
    RTK_FixType_t fix_type;     // RTK 解状态 (关键!)
    double      latitude;       // 纬度 (十进制度, double 保证厘米级精度)
    double      longitude;      // 经度 (十进制度)
    float       altitude;       // 海拔高度 (m)
    float       geoid_sep;      // 大地水准面分离 (m)
    float       hdop;           // 水平精度因子
    float       pdop;           // 位置精度因子
    float       vdop;           // 垂直精度因子
    uint8_t     satellites;     // 跟踪卫星总数
    uint8_t     gps_sats;       // GPS 卫星数
    uint8_t     bds_sats;       // BDS 卫星数
    uint8_t     glo_sats;       // GLONASS 卫星数
    uint8_t     gal_sats;       // Galileo 卫星数
    char        utc_time[12];   // UTC 时间 hhmmss.ss
    char        date[8];        // 日期 ddmmyy
} RTK_Data_t;

// 外部全局变量 (供 main.c 使用)
extern RTK_Data_t g_rtkData;
extern char rtk_line_buffer[RTK_BUF_SIZE];
extern volatile uint8_t rtk_line_ready;
extern uint8_t rtk_rx_byte;

/* ========================================
 * 函数声明
 * ======================================== */

/**
 * @brief 初始化 RTK 模块 (USART2 中断接收)
 * @return 0=成功, 1=失败
 */
uint8_t RTK_Init(void);

/**
 * @brief 主循环中轮询调用 (解析 NMEA 报文)
 */
void RTK_Process(void);

/**
 * @brief 串口中断回调 (在 main.c 的 HAL_UART_RxCpltCallback 中调用)
 */
void RTK_RxCallback(UART_HandleTypeDef *huart);

/**
 * @brief 解析 GGA 语句 (定位+质量+高程)
 */
void RTK_ParseGGA(const char *line, RTK_Data_t *data);

/**
 * @brief 解析 GSA 语句 (精度因子+卫星编号)
 */
void RTK_ParseGSA(const char *line, RTK_Data_t *data);

/**
 * @brief 解析 RMC 语句 (推荐最小定位信息)
 */
void RTK_ParseRMC(const char *line, RTK_Data_t *data);

/**
 * @brief 将 NMEA 坐标格式 (ddmm.mmmmmm) 转换为十进制度 (double 精度)
 */
double NMEA_To_Degree_Double(const char *str);

/**
 * @brief 获取 RTK 解状态字符串描述
 */
const char* RTK_GetFixTypeStr(RTK_FixType_t fix);

#endif /* __RTK_H */
