#ifndef _MotorControl_h_
#define _MotorControl_h_

//==================================================== �������Ŷ��� ====================================================
// ������ PWM ���ã���� pwm_init
#define P21_4A ATOM0_CH2_P21_4	// L
#define P21_2B ATOM1_CH0_P21_2	// L
#define P21_5A ATOM0_CH3_P21_5	// R
#define P21_3B ATOM1_CH1_P21_3	// R

// 0C��ʹ�÷�ʽ ������ ����������� zf_driver_encoder
#define P10_3_0C TIM5_ENCODER_CH1_P10_3
#define	P20_3_0C TIM6_ENCODER_CH1_P20_3

// 1C��ʹ�÷�ʽ ������ ����������� zf_driver_encoder
#define P10_1_1C TIM5_ENCODER_CH2_P10_1
#define	P20_0_1C TIM6_ENCODER_CH2_P20_0

// D��ʹ�÷�ʽ�������� ���
#define P33_9 ATOM0_CH1_P33_9

//==================================================== �������Ŷ��� ====================================================

//==================================================== �������� ====================================================
#define WIFI_SSID			"OOIT"				// WIFI����
#define WIFI_PASSWORD		"ooit1234"			// WIFI����
#define SPI_MODE			2					// SPIģʽ��1 ����λ����ͼ��2 ����λ�������������ݣ�3 ������λ��ָ�4 �շ�����
#define RUN_Mode			1					// ����ģʽ��0 ������1 ����
#define CarStop				1					// �Ƿ���ͣ����0 �رգ�1 ����
#define Find_Mode			0                   // Ѱ��ģʽ�� 0 ��Ѱ�� 1 ��Ѱ
#define DELAY_BUFFER_SIZE	1					// ת�仺��������Ƽ� 1-5����Ŀǰ����״���½���Ϊ 1

#define Max_Speed           7000				// �������ٶ��޷�
#define Set_Speed			119					// Ŀ���ٶ�
#define TurnReduceRace		0.5f				// ת�������С����

#define Kp_Speed			0.3f				// ���ٻ� Kp
#define Max_Control			50					// ���Ӽ����ٶ�
#define Kp_MotorTrun		0.2f				// ���ֲ��� Kp
#define DifferentSpeedRace	0.5f				// ���ֲ�����С����

#define Servo_TurnKp		1.6f				// ת�� ƫ�����ʱ Kp
#define Servo_NomalKp		1.0f				// ת�� ����ʱ Kp
#define Servo_NomalKd		0.0f				// ת�򻷡��ؾ໷��ת����ٻ� kd
//==================================================== �������� ====================================================

//==================================================== ��ʼ������ ====================================================
void InitElement_CPU0_OutWhile(void);
void InitElement_CPU1_OutWhile(void);
void InitElement_isr_ch0(void);
void InitElement_isr_ch1(void);
//==================================================== ��ʼ������ ====================================================

//==================================================== ������������ ====================================================
extern bool Stop_flag,OutRoad_flag, WIFI_SPI_Con;
extern int16 Left_Pulse_now, Right_Pulse_now;
extern uint8 image_copy[MT9V03X_H][MT9V03X_W];
extern int Speed_Float;
extern int Pwm1, Pwm2;
//==================================================== ������������ ====================================================
#endif
