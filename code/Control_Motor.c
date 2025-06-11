/*
 * 
 *  Name: MotorControl.c
 *  Created on: 2025/2/23
 *  Author: Magneto
 * 
 */

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
//  �������     �����ʼ������
//  ���ز���     Nope
//  ʹ��ʾ��     Motor_Init();
//  ��ע��Ϣ     Nope
//-------------------------------------------------------------------------------------------------------------------
void Motor_Init(void)
{
    pwm_init(P21_4A, 17 * 1000, 0);
    pwm_init(P21_2B, 17 * 1000, 0);
    pwm_init(P21_5A, 17 * 1000, 0);
    pwm_init(P21_3B, 17 * 1000, 0);
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     ������ƺ���
//  ���ز���     Nope
//  ʹ��ʾ��     MotorControl(1000, 1000);
//  ��ע��Ϣ     �Ƽ������Ӧ������ 6000�����Ӧ������ 10000
//-------------------------------------------------------------------------------------------------------------------
void MotorControl(int PWM_1, int PWM_2)
{
    // ��������
    if (PWM_1 >= 0) {
        pwm_set_duty(P21_2B, 0); // ����PWMռ�ձ�
        pwm_set_duty(P21_4A, PWM_1);    // ��ת�����ź�
    }
    else {
        pwm_set_duty(P21_2B, -PWM_1); // ����PWMռ�ձ�
        pwm_set_duty(P21_4A, 0);    // ��ת�����ź�
    }

    // �ҵ������
    if (PWM_2 >= 0) {
        pwm_set_duty(P21_3B, PWM_2); // ����PWMռ�ձ�
        pwm_set_duty(P21_5A, 0);    // ��ת�����ź�
    }
    else {
        pwm_set_duty(P21_3B, 0); // ����PWMռ�ձ�
        pwm_set_duty(P21_5A, -PWM_2);    // ��ת�����ź�
    }

}
//-------------------------------------------------------------------------------------------------------------------
//  �������     ���ת����ٿ��� PID
//  ���ز���     Nope
//  ʹ��ʾ��     MotorTrunReduce();
//  ��ע��Ϣ     �� PID ����ʽ
//-------------------------------------------------------------------------------------------------------------------
void MotorTrunReduce(void)
{
    // ��ʱ�м����
    uint8_t temp_CenterLine;
    // ���ת����ʹ�ý��� Error_M ֵ��Զ��С�� 90
    if (CenterLine_Average < 90) temp_CenterLine = 180 - CenterLine_Average;
    else temp_CenterLine = CenterLine_Average;
    // PID����,����� Kd ʹ����ת�򻷵� Kd
    TrunReduce.error = 90 - temp_CenterLine; // ���㵱ǰ���
    TrunReduce.out = TrunReduce.error * Kp_MotorTrun
        + (TrunReduce.error - TrunReduce.error_last) * Servo_pid_t.Dir_Kd; // ����PD���
    TrunReduce.error_last = TrunReduce.error; // ������һ�ε����

}
//-------------------------------------------------------------------------------------------------------------------
//  �������     PID���㺯��
//  ���ز���     *PiOut
//  ʹ��ʾ��     DEFINE_MOTOR_PI(L_Motorpi,true) & L_Motorpi
//  ��ע��Ϣ     true Ϊ ������false Ϊ�ҵ��
//-------------------------------------------------------------------------------------------------------------------
int MotorPi(int16* SetSpeed, int16* RealSpeed, float* Mkp, float* Mki, float* Merror, float* Merror1, int* PiOut, int16 PulseNow, bool isLeft)
{
    if (OutRoad_flag) Stop_flag = true;
    // �жϲ�����Ŀ���ٶ�
    if (isLeft) { 
        //                          ( Ŀ���ٶ� + ת����� * ��С���� + ��-�����ٶȣ�+/- ����)
        *SetSpeed = Stop_flag ? 0 : (Set_Speed + TrunReduce.out * TurnReduceRace +
                                        Speed_Float - (Servo_t.Pwm_Servo - 785) * DifferentSpeedRace);
    }
    else {
        *SetSpeed = Stop_flag ? 0 : (Set_Speed + TrunReduce.out * TurnReduceRace +
                                        Speed_Float + (Servo_t.Pwm_Servo - 785) * DifferentSpeedRace);
    }
    // ��ȡʵ���ٶ�
    *RealSpeed = PulseNow;
    // �������
    *Merror1 = *Merror;               // ������һ�����
    *Merror = *SetSpeed - *RealSpeed; // ���㵱ǰ���
    // PI����������
    *PiOut += (*Merror - *Merror1) * *Mkp + *Merror * *Mki;

    // �޷�����
    if (*PiOut > Max_Speed) {
        *PiOut = Max_Speed;
    }
    else if (*PiOut < -Max_Speed) {
        *PiOut = -Max_Speed;
    }

    return *PiOut;
}
// �жϺ���(�꺯��д������Ҫ�������Ҽ�ע��)
#define DEFINE_MOTOR_PI(func_name, dir) \
int func_name(void) {                   \
    if (dir) {                          \
        return MotorPi(LEFT_ARGS, dir); \
    } else {                            \
        return MotorPi(RIGHT_ARGS, dir); \
    }                                   \
}
// ���庯��
DEFINE_MOTOR_PI(L_Motorpi,true)
DEFINE_MOTOR_PI(R_Motorpi, false)
