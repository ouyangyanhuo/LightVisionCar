/*
 *
 *  Name: ServoControl.c
 *  Created on: 2025/2/27
 *  Author: Magneto
 *
 */

#include "zf_common_headfile.h"

 //-------------------------------------------------------------------------------------------------------------------
 //  函数简介     设置转向环 PID
 //  返回参数     Nope
 //  使用示例     Servo_Init();
 //  备注信息     该控制式虽无返回值 但会修改全局变量，动态调整 PID 需要放入中断中执行
 //-------------------------------------------------------------------------------------------------------------------
void Servo_Init(void) {
    // 舵机PID
    /*
     *  动态 Kp 当灯距离中心过远时 使 Kp 更大 加大响应
     *  直道上 Kp 更小
     *  弯道上 Kp 更大
    */
    if (CenterLine_Average < 60 || CenterLine_Average > 120)     //如果太偏离中线就增大 Kp
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
    float error = (float)((int)centerline - 90);  // 修正误差计算
    float output = Servo_pid_t->Dir_Kp * error
        + Servo_pid_t->Dir_Kd * (error - last_error);
    last_error = error;

    return (int)(output + Servo_Duty);
}
//-------------------------------------------------------------------------------------------------------------------
 //  函数简介     转向环 PID 计算式
 //  返回参数     Nope
 //  使用示例     Serorpd();
 //  备注信息     调用该函数，不断为舵机设置 PWM，需要放进中断执行
//-------------------------------------------------------------------------------------------------------------------
void Serorpd(void)
{
    uint16 temp_Pwm = PID_Servo(CenterLine_Average, &Servo_pid_t);
    // 舵机 PID 限幅
    if (temp_Pwm < Servo_Duty - 90)
        Servo_t.Pwm_Servo = Servo_Duty - 90;
    else if (temp_Pwm > Servo_Duty + 90)
        Servo_t.Pwm_Servo = Servo_Duty + 90;
    else
        Servo_t.Pwm_Servo = temp_Pwm;
    // 舵机输出 && 出界停止
    if (true == Stop_flag)
        pwm_set_duty(P33_9, 785);
    else if (Servo_t.Pwm_Servo < Servo_Duty - 90)
        pwm_set_duty(P33_9, Servo_Duty - 90);
    else if (Servo_t.Pwm_Servo > Servo_Duty + 90)
        pwm_set_duty(P33_9, Servo_Duty + 90);
    else
        pwm_set_duty(P33_9, Servo_t.Pwm_Servo); // 设置新的占空比
}
