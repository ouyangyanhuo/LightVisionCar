/*
 *
 *  Name: encoder.c
 *  Created on: 2025/2/23
 *  Author: Magneto
 *
 */
#include "zf_common_headfile.h"
 //-------------------------------------------------------------------------------------------------------------------
 //  函数简介     编码器控制函数
 //  返回参数     Nope 直接对全局变量进行操作 返回个P
 //  使用示例     Left_encoder(); / Right_encoder();
 //  备注信息     使用 宏参数化 编写，菜就别乱改！！容易爆炸
 //-------------------------------------------------------------------------------------------------------------------
#define DEFINE_ENCODER_FUNC(func_name, timer, speed_var, sign)              \
void func_name(void) {                                                      \
    /* 滤波系数 */                                                              \
    static float xi = 0.5;                                                  \
    int16_t speed = sign * encoder_get_count(timer);                        \
    /* 停车系统 */                                                              \
    xi = (Stop_flag == 1) ? 1 : xi;                                         \
    /* 上一次速度 */                                                            \
    int16_t speed_least = speed_var;                                        \
    speed_var = speed * xi + (1 - xi) * speed_least;                        \
    encoder_clear_count(timer);                                             \
}
// 具体函数
DEFINE_ENCODER_FUNC(Left_encoder, TIM5_ENCODER, Left_Pulse_now, 1)      // 左编码器，正号
DEFINE_ENCODER_FUNC(Right_encoder, TIM6_ENCODER, Right_Pulse_now, -1)   // 右编码器，负号