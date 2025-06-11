#ifndef _Control_Variable_h_
#define _Control_Variable_h_

//==================================================== �����ṹ�� ====================================================
// ���������ṹ��
typedef struct {
    int16 L_SetSpeed;    // Ŀ���ٶ�
    int16 L_RealSpeed;   // ʵ���ٶ�
    float L_Mkp;         // ����ϵ��
    float L_Mki;         // ����ϵ��
    float L_Merror;      // ��ǰ���
    float L_Merror1;     // ��һ�����
    int L_PiOut;         // PI���������
} leftpid;
extern leftpid LeftPID;
// �ҵ�������ṹ��
typedef struct {
    int16 R_SetSpeed;    // Ŀ���ٶ�
    int16 R_RealSpeed;   // ʵ���ٶ�
    float R_Mkp;         // ����ϵ��
    float R_Mki;         // ����ϵ��
    float R_Merror;      // ��ǰ���
    float R_Merror1;     // ��һ�����
    int R_PiOut;         // PI���������
} rightpid;
extern rightpid RightPID;
// �ٶȻ�ת�����
typedef struct {
    short error;
    short error_last;
    short out;
}skperror;
extern skperror TrunReduce;
// ���ٻ� PID��������ȫ��ʵ������
typedef struct {
    float Kp;            // ����ϵ��
    float Kd;            // ΢��ϵ��
    float last_error;    // ��һ�����
} PID_Controller;
//==================================================== �����ṹ�� ====================================================
// ��� PID ����
typedef struct {
    float Dir_Kp;
    float Dir_Kd;
}SERVO_PID_PARAMETERS;
extern SERVO_PID_PARAMETERS Servo_pid_t;
// ��������ṹ��
typedef struct {
    uint16 Out_Direction;			// ����������
    uint16 Pwm_Servo;				// ������
}SERVO_CONTROL;
extern SERVO_CONTROL Servo_t;

extern uint16 Servo_Duty;
#endif