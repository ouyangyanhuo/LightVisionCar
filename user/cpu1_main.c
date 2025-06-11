#include "zf_common_headfile.h"
#pragma section all "cpu1_dsram"
void core1_main(void)
{
    disable_Watchdog();                     // 关闭看门狗
    interrupt_global_enable(0);             // 打开全局中断

    InitElement_CPU1_OutWhile();

    cpu_wait_event_ready();                 // 等待所有核心初始化完毕

    pit_ms_init(CCU61_CH1, 2);

    while (TRUE)
    {
        //system_delay_ms(100);
    }
}
#pragma section all restore
