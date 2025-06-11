/*
 *
 *  Name: TCPServer.c
 *  Created on: 2025/3/1
 *  Author: Magneto
 *
 */
#include "zf_common_headfile.h"


timeoutcnt  TimeOutCnt = { .IntCnt = 0, .ConCnt = 0 };

void WIFIConnect(void) 
{
    while (wifi_spi_init(WIFI_SSID, WIFI_PASSWORD))
    {
        tft180_show_string(0, 0, "Connect failed");
        TimeOutCnt.IntCnt++;
        system_delay_ms(100);                                         // 初始化失败 等待 100ms
        // 超时结束连接
        if (TimeOutCnt.IntCnt > 10) { tft180_clear(); break; }
    }

    if (false == WIFI_SPI_AUTO_CONNECT)                               // 如果没有开启自动连接 就需要手动连接目标 IP
    {
        while (wifi_spi_socket_connect(                               // 向指定目标 IP 的端口建立 TCP 连接
            "TCP",                                                    // 指定使用TCP方式通讯
            WIFI_SPI_TARGET_IP,                                       // 指定远端的IP地址，填写上位机的IP地址
            WIFI_SPI_TARGET_PORT,                                     // 指定远端的端口号，填写上位机的端口号，通常上位机默认是8080
            WIFI_SPI_LOCAL_PORT))                                     // 指定本机的端口号
        {
            // 如果一直建立失败 考虑一下是不是没有接硬件复位
            tft180_show_string(0, 0, "Connect error");
            TimeOutCnt.ConCnt++;
            system_delay_ms(100);                                     // 建立连接失败 等待 100ms
            // 超时结束连接
            if (TimeOutCnt.ConCnt > 50 || TimeOutCnt.IntCnt > 50) { tft180_clear(); break; }
        }
    }

    if (TimeOutCnt.IntCnt >= 10) tft180_show_string(0, 120, "INT TIMEOUT");
    if (TimeOutCnt.ConCnt >= 10) tft180_show_string(0, 120, "CON TIMEOUT");

}

void DataHandleInt(void)
{
#if(1 == SPI_MODE)
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, image_copy[0], MT9V03X_W, MT9V03X_H);
#endif
}

void DataHandle(void)
{
#if(1 == SPI_MODE)
    // Copy 图像
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    // 发送图像
    seekfree_assistant_camera_send();
//#elif(2 == SPI_MODE)

#endif
}

uint8 wifi_spi_test_buffer[] = "this is wifi spi test buffer\n";
char wifi_spi_get_data_buffer[256];
uint8 wifi_sei_send_ecoder[20];
uint32 data_length;
void TrySANDRData(void) 
{
    // 发送测试数据至服务器
    sprintf(wifi_sei_send_ecoder, "%d\n", CenterLine_Average);
    wifi_spi_send_buffer(wifi_sei_send_ecoder, sizeof(wifi_sei_send_ecoder));

    //data_length = wifi_spi_read_buffer(wifi_spi_get_data_buffer, sizeof(wifi_spi_get_data_buffer));
    //if (data_length)                                                      // 如果接收到数据 则进行数据类型判断
    //{
    //    printf("\r\n Get data: <%s>.", wifi_spi_get_data_buffer);
    //    if (!wifi_spi_send_buffer(wifi_spi_get_data_buffer, data_length))
    //    {
    //        printf("\r\n send success.");
    //        memset(wifi_spi_get_data_buffer, 0, data_length);           // 数据发送完成 清空数据
    //    }
    //    else
    //    {
    //        printf("\r\n %ld bytes data send failed.", data_length);
    //    }
    //}
}
