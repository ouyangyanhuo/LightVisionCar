/*
 *
 *  Name: ServoControl.c
 *  Created on: 2025/2/27
 *  Author: Magneto
 *
 */

#include "zf_common_headfile.h"

 //-------------------------------------------------------------------------------------------------------------------
 //  �������     ����ת�� PID
 //  ���ز���     Nope
 //  ʹ��ʾ��     Servo_Init();
 //  ��ע��Ϣ     �ÿ���ʽ���޷���ֵ �����޸�ȫ�ֱ�������̬���� PID ��Ҫ�����ж���ִ��
 //-------------------------------------------------------------------------------------------------------------------
void Servo_Init(void) {
    // ���PID
    /*
     *  ��̬ Kp ���ƾ������Ĺ�Զʱ ʹ Kp ���� �Ӵ���Ӧ
     *  ֱ���� Kp ��С
     *  ����� Kp ����
    */
    if (CenterLine_Average < 60 || CenterLine_Average > 120)     //���̫ƫ�����߾����� Kp
    {
        Servo_pid_t.Dir_Kp =  Servo_TurnKp ;
    }
    else
    {
        Servo_pid_t.Dir_Kp = Servo_NomalKp;
    }
    Servo_pid_t.Dir_Kd = Servo_NomalKd;
}

uint32_t PID_Servo(uint8_t centerline, SERVO_PID_PARAMETERS* Servo_pid_t) {
    static float last_error = 0;
    float error = (float)((int)centerline - 90);  // ����������
    float output = Servo_pid_t->Dir_Kp * error
        + Servo_pid_t->Dir_Kd * (error - last_error);
    last_error = error;

    return (int)(output + Servo_Duty);
}
//-------------------------------------------------------------------------------------------------------------------
 //  �������     ת�� PID ����ʽ
 //  ���ز���     Nope
 //  ʹ��ʾ��     Serorpd();
 //  ��ע��Ϣ     ���øú���������Ϊ������� PWM����Ҫ�Ž��ж�ִ��
//-------------------------------------------------------------------------------------------------------------------
void Serorpd(void)
{
    uint16 temp_Pwm = PID_Servo(CenterLine_Average, &Servo_pid_t);
    // ��� PID �޷�
    if (temp_Pwm < Servo_Duty - 90)
        Servo_t.Pwm_Servo = Servo_Duty - 90;
    else if (temp_Pwm > Servo_Duty + 90)
        Servo_t.Pwm_Servo = Servo_Duty + 90;
    else
        Servo_t.Pwm_Servo = temp_Pwm;
    // ������ && ����ֹͣ
    if (true == Stop_flag)
        pwm_set_duty(P33_9, 785);
    else if (Servo_t.Pwm_Servo < Servo_Duty - 90)
        pwm_set_duty(P33_9, Servo_Duty - 90);
    else if (Servo_t.Pwm_Servo > Servo_Duty + 90)
        pwm_set_duty(P33_9, Servo_Duty + 90);
    else
        pwm_set_duty(P33_9, Servo_t.Pwm_Servo); // �����µ�ռ�ձ�
}
