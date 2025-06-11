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
        system_delay_ms(100);                                         // ��ʼ��ʧ�� �ȴ� 100ms
        // ��ʱ��������
        if (TimeOutCnt.IntCnt > 10) { tft180_clear(); break; }
    }

    if (false == WIFI_SPI_AUTO_CONNECT)                               // ���û�п����Զ����� ����Ҫ�ֶ�����Ŀ�� IP
    {
        while (wifi_spi_socket_connect(                               // ��ָ��Ŀ�� IP �Ķ˿ڽ��� TCP ����
            "TCP",                                                    // ָ��ʹ��TCP��ʽͨѶ
            WIFI_SPI_TARGET_IP,                                       // ָ��Զ�˵�IP��ַ����д��λ����IP��ַ
            WIFI_SPI_TARGET_PORT,                                     // ָ��Զ�˵Ķ˿ںţ���д��λ���Ķ˿ںţ�ͨ����λ��Ĭ����8080
            WIFI_SPI_LOCAL_PORT))                                     // ָ�������Ķ˿ں�
        {
            // ���һֱ����ʧ�� ����һ���ǲ���û�н�Ӳ����λ
            tft180_show_string(0, 0, "Connect error");
            TimeOutCnt.ConCnt++;
            system_delay_ms(100);                                     // ��������ʧ�� �ȴ� 100ms
            // ��ʱ��������
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
    // Copy ͼ��
    memcpy(image_copy[0], mt9v03x_image[0], MT9V03X_IMAGE_SIZE);
    // ����ͼ��
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
    // ���Ͳ���������������
    sprintf(wifi_sei_send_ecoder, "%d\n", CenterLine_Average);
    wifi_spi_send_buffer(wifi_sei_send_ecoder, sizeof(wifi_sei_send_ecoder));

    //data_length = wifi_spi_read_buffer(wifi_spi_get_data_buffer, sizeof(wifi_spi_get_data_buffer));
    //if (data_length)                                                      // ������յ����� ��������������ж�
    //{
    //    printf("\r\n Get data: <%s>.", wifi_spi_get_data_buffer);
    //    if (!wifi_spi_send_buffer(wifi_spi_get_data_buffer, data_length))
    //    {
    //        printf("\r\n send success.");
    //        memset(wifi_spi_get_data_buffer, 0, data_length);           // ���ݷ������ �������
    //    }
    //    else
    //    {
    //        printf("\r\n %ld bytes data send failed.", data_length);
    //    }
    //}
}
