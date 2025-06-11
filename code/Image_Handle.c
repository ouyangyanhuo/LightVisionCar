/*
 *
 *  Name: Image_Handle.c
 *  Created on: 2025/2/24
 *  Author: Magneto
 *
 */

#include "zf_common_headfile.h"
//-------------------------------------------------------------------------------------------------------------------
//  �������     ���ƣ����������ұ߽�
//-------------------------------------------------------------------------------------------------------------------
// Ѱ���ϱ߽�
int Find_Top_Edge(void)
{
    for (uint8_t y = 0; y < 60; y++)
    {
        for (uint8_t x = 0; x < 128; x++) {
            if (mt9v03x_image[y][x] == Edge_Range)
            {
                return y;
            }
        }
    }
    return 0;
}
// Ѱ���±߽�
int Find_Bottom_Edge(void)
{
    for (uint8_t y = 59; y > 0; y--)
    {
        for (uint8_t x = 0; x < 128; x++) {
            if (mt9v03x_image[y][x] == Edge_Range)
            {
                return y;
            }
        }
    }
    return 59;
}
// Ѱ����߽�
int Find_Left_Edge(void)
{
    for (uint8_t x = 0; x < 128; x++)
    {
        for (uint8_t y = 0; y < 60; y++) {
            if (mt9v03x_image[y][x] == Edge_Range)
            {
                return x;
            }
        }
    }
    return 0;
}
// Ѱ���ұ߽�
int Find_Right_Edge(void)
{
    for (uint8_t x = 127; x > 0; x--)
    {
        for (uint8_t y = 0; y < 60; y++) {
            if (mt9v03x_image[y][x] == Edge_Range)
            {
                return x;
            }
        }
    }
    return 127;
}
// Ѱ���ڱ߽纯��
InnerEdgeResult Find_Inner_Edges_Optimized(int top, int bottom, int left, int right)
{
    InnerEdgeResult result;
#if 0 == Find_Mode
    // Ϊ�ӿ��յ����ʣ���ģʽ�²�ʹ�������ڱ�Ե��⣬��ʼ��Ϊ 0 ����
    result.inner_top = 0;
    result.inner_bottom = 0;
    // ������ұ�Ե
    int center = (left + right) / 2;

    // ɨ�����߾͸��ҹ�����ֵȥ
    result.inner_left = center;
    result.inner_right = center;

    // ����ɨ��������߽�
    for (int x = center; x > left; x--) {
        for (int y = 0; y < 60; y++) {
            if (mt9v03x_image[y][x] == Edge_Range) {
                result.inner_left = x;
                goto found_left;
            }
        }
    }
found_left:
    for (int x = center; x < right; x++) {
        for (int y = 0; y < 60; y++) {
            if (mt9v03x_image[y][x] == Edge_Range) {
                result.inner_right = x;
                goto found_right;
            }
        }
    }
found_right:
#else
    // Ϊ�ӿ��յ����ʣ���ģʽ�²�ʹ�������ڱ�Ե��⣬��ʼ��Ϊ 0 ����
    result.inner_left = 0;
    result.inner_right = 0;
    // ����������Ե
    int center = (top + bottom) / 2;

    // ɨ�����߾͸��ҹ�����ֵȥ
    result.inner_top = center;
    result.inner_bottom = center;

    for (int y = center; y > top; y--) {
        for (int x = 0; x < 128; x++) {
            if (mt9v03x_image[y][x] == Edge_Range) {
                result.inner_top = y;
                goto found_top;
            }
        }
    }
found_top:
    for (int y = center; y < bottom; y++) {
        for (int x = 0; x < 128; x++) {
            if (mt9v03x_image[y][x] == Edge_Range) {
                result.inner_bottom = y;
                goto found_bottom;
            }
        }
    }
found_bottom:
#endif
    return result;
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     �����߼�����ǿ
//-------------------------------------------------------------------------------------------------------------------
float Calculate_Enhanced_CenterLine(int left, int inner_left, int right, int inner_right)
{
    // ԭʼ�߽����ļ���
    float outer_center = ((float)(left + right)) / 2.0f;
#if 0 == Find_Mode
    // �ڱ߽����ļ���
    float inner_center = ((float)(inner_left + inner_right)) / 2.0f;
    // ��̬Ȩ�أ��������߽����󣨼�ת�䣩�������ڱ߽�Ȩ��
    float boundary_diff = fabs(inner_left - inner_right);
    float inner_weight = (boundary_diff > 30.0f) ? 0.1f : 0.3f; // �����ʱȨ�ؽ���10%
    float weighted_center = outer_center * (1.0f - inner_weight) + inner_center * inner_weight;
#else
    float weighted_center = outer_center;
#endif
    // ת��Ϊ�Ƕȣ�0-180�㷶Χ������ת����
    float CenterLine_Angle = 180.0f - (weighted_center / 128.0f * 180.0f);

    return CenterLine_Angle;
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     �������˲��ṹ��ȫ��ʵ����
//-------------------------------------------------------------------------------------------------------------------
KalmanFilter kf_centerline;
//-------------------------------------------------------------------------------------------------------------------
//  �������     �������˲�����ʼ��
//-------------------------------------------------------------------------------------------------------------------
// ��ʼ���������˲������ڳ����ʼ��ʱ���ã�
void Kalman_Init(void)
{
    kf_centerline.q = 0.5f;     // ����������ԽСԽ����Ԥ�⣩ʹ�˲���������ʷԤ�⣨�ͺ�
    kf_centerline.r = 0.01f;    // �۲�������ԽСԽ���β�����
    kf_centerline.p = 1.0f;     // ��ʼ�������
    kf_centerline.x = 90.0f;    // ��ʼ�Ƕȣ�ͼ�����У�
    kf_centerline.last_angle = 90.0f;
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     �������˲�������
//-------------------------------------------------------------------------------------------------------------------
float Kalman_Update(float measurement)
{
    // Ԥ��׶�
    kf_centerline.p = kf_centerline.p + kf_centerline.q;

    // ���½׶�
    kf_centerline.k = kf_centerline.p / (kf_centerline.p + kf_centerline.r);
    kf_centerline.x = kf_centerline.x + kf_centerline.k * (measurement - kf_centerline.x);
    kf_centerline.p = (1 - kf_centerline.k) * kf_centerline.p;

    return kf_centerline.x;
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     �������кϼ���
//-------------------------------------------------------------------------------------------------------------------
float angle_buffer[DELAY_BUFFER_SIZE] = { 90.0f };
uint8_t buffer_index = 0;
void CarRun(int top, int bottom, int left, int new_left, int right, int new_right) {

#if 1 == CarStop
#if 0 == Find_Mode
    if (59 == bottom && 0 == left && 127 == right && 63 == new_left && 63 == new_right)
        LostLight_cnt++;
#else
    if (59 == bottom && 0 == left && 127 == right && 0 == new_left && 0 == new_right)
        LostLight_cnt++;
#endif
#endif
    if (LostLight_cnt > 10) {
        Stop_flag = true;
        LostLight_cnt = 0;
    }
    else
    {
        // ��Ȩ����
        float enhanced_centerline = Calculate_Enhanced_CenterLine(left, new_left, right, new_right);
        // ʹ�ÿ������˲�������ʹ����ǿ�����ߣ�
        float filtered_angle = Kalman_Update(enhanced_centerline);

        // ����ǰ�Ƕȴ��뻺����
        angle_buffer[buffer_index] = filtered_angle;
        buffer_index = (buffer_index + 1) % DELAY_BUFFER_SIZE;
        // ʹ�û���������ɵĽǶȣ��ӳ�ת�䣩
        CenterLine_Average = angle_buffer[buffer_index];
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     �����Ƶı߽�
//-------------------------------------------------------------------------------------------------------------------
void DrawEdge(int top, int new_top, int bottom, int new_bottom, int left, int new_left, int right, int new_right) {
#if 0 == Find_Mode
    for (uint8_t y = 0; y < 60; y++)
    {
        tft180_draw_point(left, y, RGB565_BLUE);
        tft180_draw_point(right, y, RGB565_GREEN);

        tft180_draw_point(new_left, y, RGB565_MAGENTA);
        tft180_draw_point(new_right, y, RGB565_CYAN);
    }
    for (uint8_t x = 0; x < 128; x++)
    {
        tft180_draw_point(x, top, RGB565_RED);
        tft180_draw_point(x, bottom, RGB565_YELLOW);
    }
#else
    for (uint8_t y = 0; y < 60; y++)
    {
        tft180_draw_point(left, y, RGB565_BLUE);
        tft180_draw_point(right, y, RGB565_GREEN);
    }
    for (uint8_t x = 0; x < 128; x++)
    {
        tft180_draw_point(x, top, RGB565_RED);
        tft180_draw_point(x, bottom, RGB565_YELLOW);

        tft180_draw_point(x, new_top, RGB565_MAGENTA);
        tft180_draw_point(x, new_bottom, RGB565_CYAN);
    }
#endif
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     �ٶȾ��� �� ����
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  �������     PID ��������ʼ��
//  ����˵��     kp: ����ϵ��, kd: ΢��ϵ����kd ʹ�õ���ת���
//  ���ز���     ��ʼ���õ� PID ������
//-------------------------------------------------------------------------------------------------------------------
PID_Controller PID_Init(float kp, float kd) {
    PID_Controller pid;
    pid.Kp = kp;
    pid.Kd = kd;
    pid.last_error = 0;
    return pid;
}

//-------------------------------------------------------------------------------------------------------------------
//  �������     ���� PID ���
//  ����˵��     pid: PID ������, error: ��ǰ����Χ��0-63��
//  ���ز���     PID ���ֵ����Χ��0-Max_Control����ֵԽ���ʾ�ٶ�Խ����ԽС�ٶ�Խ��
//-------------------------------------------------------------------------------------------------------------------
float PID_Compute(PID_Controller* pid, float error) {
    // PD ���ƹ�ʽ����� = Kp * error + Kd * derivative
    float output = pid->Kp * error + pid->Kd * (error - pid->last_error);
    pid->last_error = error;  // ������һ�����

    // �������ֵ�� 0 �� Max_Control ֮��
    if (output > Max_Control) {
        output = Max_Control;
    }
    else if (output < 0.0f) {
        output = 0.0f;
    }
    return output;
}
void SpeedControl(int top, int bottom, int left, int right) {
    int Center = right - left;
    // ����
    if (Center != 127) {
        Stop_flag = false;
    }
    PID_Controller pid = PID_Init(Kp_Speed, Servo_pid_t.Dir_Kd);                //�ؾ໷pid
#if 0 == Find_Mode
    float control_signal = PID_Compute(&pid, Center);
#else
    int Center_High = bottom - top;
    float control_signal = PID_Compute(&pid, Center_High);
#endif
    Speed_Float = -control_signal;
}
//-------------------------------------------------------------------------------------------------------------------
//  �������     ���մ���
//-------------------------------------------------------------------------------------------------------------------
void ImageProcess(void)
{
    // ��ȡ�߽�
    int top = Find_Top_Edge();
    int bottom = Find_Bottom_Edge();
    int left = Find_Left_Edge();
    int right = Find_Right_Edge();
    // ����ɨ�߽�
    InnerEdgeResult inner_edges = Find_Inner_Edges_Optimized(top, bottom, left, right);
    // ��ȡ�ڱ߽�ֵ
    int new_left = inner_edges.inner_left;
    int new_right = inner_edges.inner_right;
#if 1 == Find_Mode
    int new_top = inner_edges.inner_top;
    int new_bottom = inner_edges.inner_bottom;
#endif
    // ��������
    CarRun(top, bottom, left, new_left, right, new_right);
    // �ٶȾ����뷢��
    SpeedControl(top, bottom, left, right);
    // DEBUG: �����߽�
    DrawTop = top;
    DrawBottom = bottom;
    DrawLeft = left;
    DrawLeft2 = new_left;
    DrawRight = right;
    DrawRight2 = new_right;
#if 0 == Find_Mode
    DrawTop2 = 0;
    DrawBottom2 = 0;
#else
    DrawTop2 = new_top;
    DrawBottom2 = new_bottom;
#endif

    tft180_show_int(0, 80, bottom - top, 2);
}
