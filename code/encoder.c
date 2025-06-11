/*
 *
 *  Name: encoder.c
 *  Created on: 2025/2/23
 *  Author: Magneto
 *
 */
#include "zf_common_headfile.h"
 //-------------------------------------------------------------------------------------------------------------------
 //  �������     ���������ƺ���
 //  ���ز���     Nope ֱ�Ӷ�ȫ�ֱ������в��� ���ظ�P
 //  ʹ��ʾ��     Left_encoder(); / Right_encoder();
 //  ��ע��Ϣ     ʹ�� ������� ��д���˾ͱ��Ҹģ������ױ�ը
 //-------------------------------------------------------------------------------------------------------------------
#define DEFINE_ENCODER_FUNC(func_name, timer, speed_var, sign)              \
void func_name(void) {                                                      \
    /* �˲�ϵ�� */                                                              \
    static float xi = 0.5;                                                  \
    int16_t speed = sign * encoder_get_count(timer);                        \
    /* ͣ��ϵͳ */                                                              \
    xi = (Stop_flag == 1) ? 1 : xi;                                         \
    /* ��һ���ٶ� */                                                            \
    int16_t speed_least = speed_var;                                        \
    speed_var = speed * xi + (1 - xi) * speed_least;                        \
    encoder_clear_count(timer);                                             \
}
// ���庯��
DEFINE_ENCODER_FUNC(Left_encoder, TIM5_ENCODER, Left_Pulse_now, 1)      // �������������
DEFINE_ENCODER_FUNC(Right_encoder, TIM6_ENCODER, Right_Pulse_now, -1)   // �ұ�����������