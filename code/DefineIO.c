/*
 *
 *  Name: Define.c
 *  Created on: 2025/2/23
 *  Author: Magneto
 *
 */
#include "zf_common_headfile.h"

 //==================================================== �������� ====================================================
 // ͣ����־λ��Ĭ��Ϊ 0 (false δͣ��)
bool Stop_flag = false;
// ��������־λ��Ĭ��Ϊ 0 (false δ����)
bool OutRoad_flag = false;
// ����WIFI���ӣ�Ĭ��Ϊ 1 (true ����)
bool WIFI_SPI_Con = false;
// ���ҵ����������-ʵ�ʶ���
int16 Left_Pulse_now = 0, Right_Pulse_now = 0;
// ����PWM����
int Pwm1 = 0, Pwm2 = 0;
// ���Ƶ�ͼ�����ݣ�����WiFi���䣩
uint8 image_copy[MT9V03X_H][MT9V03X_W];
// ���ٻ� �ӿ�
int Speed_Float = 0;
//==================================================== �������� ====================================================

//-------------------------------------------------------------------------------------------------------------------
//  �������     CPU0���������� While ��������ʼ������
//  ���ز���     Nope
//  ʹ��ʾ��     InitElement_CPU0_OutWhile();
//  ��ע��Ϣ     Nope
//-------------------------------------------------------------------------------------------------------------------
void InitElement_CPU0_OutWhile(void)
{
	// �����ʼ��
	Motor_Init();
	// ��ͷ��ʼ��
	mt9v03x_init();
	// ��������ʼ��
	encoder_dir_init(TIM5_ENCODER, P10_3_0C, P10_1_1C);
	encoder_dir_init(TIM6_ENCODER, P20_3_0C, P20_0_1C);
	// �����ʼ��
	pwm_init(P33_9, 50, Servo_Duty);
	// �������˲�������ʼ��
	Kalman_Init();
	// SPI ����ͨ�Ŵ���
	if (true == WIFI_SPI_Con) WIFIConnect();
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     �ж�ϵͳ���������� ch0 �������ʼ������
//  ���ز���     Nope
//  ʹ��ʾ��     InitElement_isr_ch0();
//  ��ע��Ϣ     Nope
//-------------------------------------------------------------------------------------------------------------------

void InitElement_isr_ch0(void)
{
	Servo_Init();                         //���ڴ˴���Ϊ�˶�̬pid
	// ���ұ�������ȡ��������
	Left_encoder();
	Right_encoder();
	// ��� PID ����
	Serorpd();
	// ��� PID ����
	MotorTrunReduce();
	// �������
	//	MotorControl(1600, 1600); //�������
	Pwm1 = L_Motorpi();
	Pwm2 = R_Motorpi();
	MotorControl(Pwm1, Pwm2);
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     CPU1���������� While ��������ʼ������
//  ���ز���     Nope
//  ʹ��ʾ��     InitElement_CPU1_OutWhile();
//  ��ע��Ϣ     Nope
//-------------------------------------------------------------------------------------------------------------------
void InitElement_CPU1_OutWhile(void)
{
	// SPI ����ͨ�ų�ʼ��
	if (true == WIFI_SPI_Con) seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     �ж�ϵͳ���������� ch1 �������ʼ������
//  ���ز���     Nope
//  ʹ��ʾ��     InitElement_isr_ch1();
//  ��ע��Ϣ     Nope
//-------------------------------------------------------------------------------------------------------------------
int Show_Cnt=0;
void InitElement_isr_ch1(void)
{
	if (true == WIFI_SPI_Con) DataHandleInt();
	if (mt9v03x_finish_flag) {
		tft180_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
		mt9v03x_finish_flag = 0;
		if (true == WIFI_SPI_Con) DataHandle();
	}
	ImageProcess();

	// ��Ļ������ʾ
	// EncoderData();

//	Show_Cnt++;
//	if(Show_Cnt >=80){
//		tft180_show_int(0, 120, Pwm1, 4);
//		tft180_show_int(50, 120, Pwm2, 4);
//		tft180_show_int(0, 60, Left_Pulse_now, 3);
//		tft180_show_int(50, 60, Right_Pulse_now, 3);
//		Show_Cnt = 0;
//	}
//
//	DrawEdge(DrawTop, DrawTop2, DrawBottom, DrawBottom2, DrawLeft, DrawLeft2, DrawRight, DrawRight2);
//	tft180_show_float(0, 140, Servo_t.Pwm_Servo, 3, 1);
//  tft180_show_float(60, 140, CenterLine_Average, 3, 1);
}
