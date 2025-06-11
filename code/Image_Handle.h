#ifndef _IMAGE_HANDLE_H_
#define _IMAGE_HANDLE_H_

// ɨ�߽ṹ��
typedef struct {
    int inner_left;
    int inner_right;
    int inner_top;
    int inner_bottom;
} InnerEdgeResult;
// �������˲����ṹ��
typedef struct {
    float q;        // ��������Э����
    float r;        // �۲�����Э����
    float p;        // �������Э����
    float k;        // ����������
    float x;        // ״ֵ̬���˲���������߽Ƕȣ�
    float last_angle; // �ϴνǶȣ����ڱ仯�ʼ��㣩
} KalmanFilter;

void ImageProcess(void);
int Find_Top_Edge(void);
int Find_Bottom_Edge(void);
int Find_Left_Edge(void);
int Find_Right_Edge(void);
void SpeedControl(int top, int bottom, int left, int right);
InnerEdgeResult Find_Inner_Edges_Optimized(int top, int bottom, int left, int right);
float Calculate_Enhanced_CenterLine(int left, int inner_left, int right, int inner_right);
void Kalman_Init(void);
float Kalman_Update(float measurement);
void DrawEdge(int top, int new_top, int bottom, int new_bottom, int left, int new_left, int right, int new_right);

#endif