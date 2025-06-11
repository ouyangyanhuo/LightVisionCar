#include "zf_common_headfile.h"
#pragma section all "cpu0_dsram"

int core0_main(void)
{
    clock_init();                   // ��ȡʱ��Ƶ��<��ر���>
    debug_init();                   // ��ʼ��Ĭ�ϵ��Դ���

    // ��Ļ��ʼ��
    tft180_init();
    if (true == WIFI_SPI_Con) tft180_show_string(0, 80, "Waitting Connect");
    else tft180_show_string(40, 80, "Welcome");

    InitElement_CPU0_OutWhile();

    tft180_clear();
    cpu_wait_event_ready();         // �ȴ����к��ĳ�ʼ�����
    pit_ms_init(CCU60_CH0, 2);
    while (TRUE)
    {
        
        //TrySANDRData();
        //system_delay_ms(1000);
    }
}

#pragma section all restore
