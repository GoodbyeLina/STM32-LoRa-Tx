#include "gps.h"

// 变量定义
char gps_line_buffer[GPS_BUF_SIZE];
uint8_t gps_line_ready = 0;
uint8_t gps_buf_index = 0;
uint8_t gps_rx_byte = 0;
GPS_Data_t g_gpsData = {0};

/**
 * @brief 初始化 GPS 接收
 */
uint8_t GPS_Init(void) {
	
		HAL_StatusTypeDef status;
	
    memset(gps_line_buffer, 0, GPS_BUF_SIZE);
    gps_line_ready = 0;
    gps_buf_index = 0;
    // 启动第一次中断接收
		status = HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
		if(status != HAL_OK){
			printf("GPS init failed!\r\n");
			return 1;
		} else {
			printf("GPS init success!\r\n");
			return 0;
		}
			
}

// 辅助函数：将 NMEA 的 ddmm.mmmm 格式转换为十进制度
float NMEA_To_Degree(char *str) {
    if (strlen(str) < 5) return 0.0f;
    float raw = atof(str);
    int degrees = (int)(raw / 100);
    float minutes = raw - (degrees * 100);
    return degrees + (minutes / 60.0f);
}	

/**
 * @brief 在主循环中调用的解析函数
 */
void GPS_Process(void) {
    if (gps_line_ready) {
        // 寻找 RMC 语句 (包含最基本的定位状态)
        char *rmc_ptr = strstr(gps_line_buffer, "RMC");
        
				if (rmc_ptr != NULL) {
            // 检查定位状态位 (RMC 语句第 2 个逗号后)
            if (strstr(gps_line_buffer, ",A,")) {
                g_gpsData.is_valid = 1;
                // 简单的逗号计数解析（实际项目中建议使用更严谨的 strtok）
                char *p = gps_line_buffer;
                int comma_cnt = 0;
                while(*p) {
                    if(*p == ',') {
                        comma_cnt++;
                        if(comma_cnt == 3) g_gpsData.latitude = NMEA_To_Degree(p+1);
                        if(comma_cnt == 5) g_gpsData.longitude = NMEA_To_Degree(p+1);
                    }
                    p++;
                }
                printf("[GPS] 定位成功: Lat:%.4f, Lon:%.4f\r\n", g_gpsData.latitude, g_gpsData.longitude);
            } else {
                g_gpsData.is_valid = 0;
                printf("[GPS] 正在搜星...\r\n");
            }
        }

        // 处理完毕，重置状态
        gps_line_ready = 0;
        gps_buf_index = 0;
    }
}
/**
 * @brief 串口中断回调处理逻辑
 * 将此函数放在 main.c 的 HAL_UART_RxCpltCallback 中调用
 */
void GPS_Callback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) { // 确保是 GPS 连接的串口
        if (gps_buf_index < GPS_BUF_SIZE - 1) {
            gps_line_buffer[gps_buf_index++] = gps_rx_byte;
            
            // 检测到换行符，说明一条指令结束
            if (gps_rx_byte == '\n') {
                gps_line_buffer[gps_buf_index] = '\0';
                gps_line_ready = 1;
            }
        } else {
            gps_buf_index = 0; // 溢出保护
        }

        // 继续开启下一次中断接收
        HAL_UART_Receive_IT(&huart2, &gps_rx_byte, 1);
    }
}
