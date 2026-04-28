#include "rtk.h"

/* ========================================
 * 全局变量定义
 * ======================================== */
RTK_Data_t g_rtkData = {0};                     // RTK 数据全局实例
char rtk_line_buffer[RTK_BUF_SIZE];              // NMEA 行缓冲区
volatile uint8_t rtk_line_ready = 0;             // 行就绪标志
static uint16_t rtk_buf_index = 0;               // 缓冲区索引
uint8_t rtk_rx_byte = 0;                         // 中断接收单字节

/* ========================================
 * 内部辅助函数
 * ======================================== */

/**
 * @brief 检查 NMEA 校验和
 * @param line NMEA 语句 (含 $ 和 *XX)
 * @return 1=校验通过, 0=失败
 */
static uint8_t NMEA_CheckChecksum(const char *line) {
    if (line[0] != '$') return 0;
    
    // 找到 '*' 位置
    const char *asterisk = strchr(line, '*');
    if (!asterisk) return 0;
    
    // 计算 $ 和 * 之间的异或值
    uint8_t calc = 0;
    const char *p = line + 1; // 跳过 $
    while (p < asterisk) {
        calc ^= *p++;
    }
    
    // 提取报文中的校验和 (十六进制)
    uint16_t recv = (uint16_t)strtol(asterisk + 1, NULL, 16);
    
    return (calc == (uint8_t)recv);
}

/**
 * @brief 从 NMEA 语句中按逗号索引提取字段
 * @param line 原始 NMEA 语句
 * @param field_index 字段索引 (从 0 开始)
 * @param out 输出缓冲区
 * @param max_len 输出缓冲区最大长度
 * @return 字段字符串指针, 失败返回 NULL
 */
static const char* GetNMEAField(const char *line, int field_index, char *out, int max_len) {
    int current = 0;
    const char *p = line;
    
    // 跳过 $
    if (*p == '$') p++;
    
    while (*p) {
        if (*p == ',') {
            current++;
            if (current > field_index) {
                // 提取当前字段
                int len = 0;
                p++;
                while (*p && *p != ',' && *p != '*' && len < max_len - 1) {
                    out[len++] = *p++;
                }
                out[len] = '\0';
                return out;
            }
        }
        if (*p == '*') break; // 到了校验和部分
        p++;
    }
    return NULL; // 字段不存在
}

/* ========================================
 * 坐标转换
 * ======================================== */

double NMEA_To_Degree_Double(const char *str) {
    if (!str || strlen(str) < 5) return 0.0;
    
    double raw = atof(str);
    int degrees = (int)(raw / 100.0);
    double minutes = raw - (degrees * 100.0);
    
    return degrees + (minutes / 60.0);
}

/* ========================================
 * RTK 解状态字符串
 * ======================================== */

const char* RTK_GetFixTypeStr(RTK_FixType_t fix) {
    switch (fix) {
        case RTK_FIX_INVALID:    return "Invalid";
        case RTK_FIX_SINGLE:     return "Single";
        case RTK_FIX_DGPS:       return "DGPS";
        case RTK_FIX_PPS:        return "PPS";
        case RTK_FIX_FIXED:      return "⭐RTK Fixed";
        case RTK_FIX_FLOAT:      return "RTK Float";
        case RTK_FIX_DEAD_RECK:  return "Dead Reck";
        case RTK_FIX_MANUAL:     return "Manual";
        case RTK_FIX_SIMULATION: return "Sim";
        default:                 return "Unknown";
    }
}

/* ========================================
 * NMEA 解析器
 * ======================================== */

void RTK_ParseGGA(const char *line, RTK_Data_t *data) {
    // $GNGGA,time,lat,NS,lon,EW,quality,numSats,HDOP,alt,M,sep,M,age,refID*CS
    //   0     1    2  3  4   5   6      7       8   9 10 11 12  13   14
    char field[32];
    
    // 字段 1: UTC 时间
    if (GetNMEAField(line, 1, field, sizeof(field))) {
        strncpy(data->utc_time, field, 11);
        data->utc_time[10] = '\0';
    }
    
    // 字段 2: 纬度
    if (GetNMEAField(line, 2, field, sizeof(field))) {
        data->latitude = NMEA_To_Degree_Double(field);
    }
    
    // 字段 3: N/S
    if (GetNMEAField(line, 3, field, sizeof(field))) {
        if (field[0] == 'S') data->latitude = -data->latitude;
    }
    
    // 字段 4: 经度
    if (GetNMEAField(line, 4, field, sizeof(field))) {
        data->longitude = NMEA_To_Degree_Double(field);
    }
    
    // 字段 5: E/W
    if (GetNMEAField(line, 5, field, sizeof(field))) {
        if (field[0] == 'W') data->longitude = -data->longitude;
    }
    
    // 字段 6: 定位质量 (⭐ RTK 解状态)
    if (GetNMEAField(line, 6, field, sizeof(field))) {
        data->fix_type = (RTK_FixType_t)atoi(field);
    }
    
    // 字段 7: 卫星数
    if (GetNMEAField(line, 7, field, sizeof(field))) {
        data->satellites = (uint8_t)atoi(field);
    }
    
    // 字段 8: HDOP
    if (GetNMEAField(line, 8, field, sizeof(field))) {
        data->hdop = atof(field);
    }
    
    // 字段 9: 海拔高度
    if (GetNMEAField(line, 9, field, sizeof(field))) {
        data->altitude = atof(field);
    }
    
    // 字段 11: 大地水准面分离
    if (GetNMEAField(line, 11, field, sizeof(field))) {
        data->geoid_sep = atof(field);
    }
    
    // 判断是否有效定位
    data->is_valid = (data->fix_type >= RTK_FIX_SINGLE) ? 1 : 0;
}

void RTK_ParseGSA(const char *line, RTK_Data_t *data) {
    // $GNGSA,autoMode,fixMode,sat1,...,sat12,PDOP,HDOP,VDOP*CS
    //   0       1        2      3~14   15   16   17
    char field[16];
    char mode;
    
    // 字段 1: 定位模式 (1=无, 2=2D, 3=3D)
    if (GetNMEAField(line, 2, field, sizeof(field))) {
        mode = field[0];
    }
    
    // 字段 15: PDOP
    if (GetNMEAField(line, 15, field, sizeof(field))) {
        data->pdop = atof(field);
    }
    
    // 字段 16: HDOP (也用 GSA 的 HDOP 覆盖，更准确)
    if (GetNMEAField(line, 16, field, sizeof(field))) {
        data->hdop = atof(field);
    }
    
    // 字段 17: VDOP
    if (GetNMEAField(line, 17, field, sizeof(field))) {
        data->vdop = atof(field);
    }
}

void RTK_ParseRMC(const char *line, RTK_Data_t *data) {
    // $GNRMC,time,status,lat,NS,lon,EW,speed,course,date,magVar,magDir*CS
    //   0     1    2     3  4  5  6   7     8     9    10    11
    char field[16];
    
    // 字段 2: 状态 (A=有效, V=无效)
    if (GetNMEAField(line, 2, field, sizeof(field))) {
        if (field[0] == 'A') {
            data->is_valid = 1;
        }
    }
    
    // 字段 3: 纬度 (可用作补充)
    // (GGA 中已经解析了经纬度，RMC 的可作为备选)
    
    // 字段 9: 日期
    if (GetNMEAField(line, 9, field, sizeof(field))) {
        strncpy(data->date, field, 6);
        data->date[6] = '\0';
    }
}

/* ========================================
 * 公共接口
 * ======================================== */

uint8_t RTK_Init(void) {
    HAL_StatusTypeDef status;
    
    // 清空缓冲区
    memset(rtk_line_buffer, 0, RTK_BUF_SIZE);
    rtk_line_ready = 0;
    rtk_buf_index = 0;
    
    // 清空数据
    memset(&g_rtkData, 0, sizeof(RTK_Data_t));
    
    // 启动 USART2 中断接收 (1 字节)
    status = HAL_UART_Receive_IT(&huart2, &rtk_rx_byte, 1);
    
    if (status != HAL_OK) {
        printf("[RTK] Init FAILED! (UART2 IT start error)\r\n");
        return 1;
    }
    
    printf("[RTK] Init OK! Waiting for UM982 data...\r\n");
    return 0;
}

void RTK_RxCallback(UART_HandleTypeDef *huart) {
    // 注意: 这里参数类型是为了兼容性, 实际调用时传入 &huart2
    if (huart->Instance == USART2) {
        // 1. 存储字节到行缓冲区
        if (rtk_buf_index < RTK_BUF_SIZE - 1) {
            rtk_line_buffer[rtk_buf_index++] = rtk_rx_byte;
            
            // 2. 检测行结束 (\n)
            if (rtk_rx_byte == '\n') {
                rtk_line_buffer[rtk_buf_index] = '\0';  // 字符串结束
                rtk_line_ready = 1;                       // 标记行就绪
            }
        } else {
            // 溢出保护: 重置缓冲区
            rtk_buf_index = 0;
            memset(rtk_line_buffer, 0, RTK_BUF_SIZE);
        }
        
        // 3. 继续接收下一个字节
        HAL_UART_Receive_IT(&huart2, &rtk_rx_byte, 1);
    }
}

void RTK_Process(void) {
    if (!rtk_line_ready) return;
    
    rtk_line_ready = 0;  // 立即清除标志 (防止重入)
    
    // 可选: 校验 NMEA 校验和 (调试时可先关闭, 确认数据正常后再开启)
    // if (!NMEA_CheckChecksum(rtk_line_buffer)) {
    //     rtk_buf_index = 0;
    //     return;  // 校验和失败, 丢弃
    // }
    
    // 判断语句类型并分发解析
    if (strstr(rtk_line_buffer, "GGA")) {
        RTK_ParseGGA(rtk_line_buffer, &g_rtkData);
    }
    else if (strstr(rtk_line_buffer, "GSA")) {
        RTK_ParseGSA(rtk_line_buffer, &g_rtkData);
    }
    else if (strstr(rtk_line_buffer, "RMC")) {
        RTK_ParseRMC(rtk_line_buffer, &g_rtkData);
    }
    
    // 打印调试信息 (每收到一条 GGA 就打印一次状态)
    if (strstr(rtk_line_buffer, "GGA")) {
        printf("\r\n[RTK] %s | Sat:%d | %s\r\n", 
               RTK_GetFixTypeStr(g_rtkData.fix_type),
               g_rtkData.satellites,
               g_rtkData.utc_time);
        printf("[RTK] Lat:%.7f, Lon:%.7f, Alt:%.2fm\r\n",
               g_rtkData.latitude,
               g_rtkData.longitude,
               g_rtkData.altitude);
        printf("[RTK] HDOP:%.1f, PDOP:%.1f, VDOP:%.1f\r\n",
               g_rtkData.hdop,
               g_rtkData.pdop,
               g_rtkData.vdop);
    }
    
    // 重置缓冲区索引
    rtk_buf_index = 0;
}
