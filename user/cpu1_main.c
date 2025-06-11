#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
void core1_main(void)
{
    disable_Watchdog();                     // �رտ��Ź�
    interrupt_global_enable(0);             // ��ȫ���ж�

    InitElement_CPU1_OutWhile();

    cpu_wait_event_ready();                 // �ȴ����к��ĳ�ʼ�����

    pit_ms_init(CCU61_CH1, 2);

    while (TRUE)
    {
        //system_delay_ms(100);
    }
}
#pragma section all restore
