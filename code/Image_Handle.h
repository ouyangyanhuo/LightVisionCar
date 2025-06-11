#ifndef _IMAGE_HANDLE_H_
#define _IMAGE_HANDLE_H_

// 扫线结构体
typedef struct {
    int inner_left;
    int inner_right;
    int inner_top;
    int inner_bottom;
} InnerEdgeResult;
// 卡尔曼滤波器结构体
typedef struct {
    float q;        // 过程噪声协方差
    float r;        // 观测噪声协方差
    float p;        // 估计误差协方差
    float k;        // 卡尔曼增益
    float x;        // 状态值（滤波后的中心线角度）
    float last_angle; // 上次角度（用于变化率计算）
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