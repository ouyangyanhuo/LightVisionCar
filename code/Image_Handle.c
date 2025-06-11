/*
 *
 *  Name: Image_Handle.c
 *  Created on: 2025/2/24
 *  Author: Magneto
 *
 */

#include "zf_common_headfile.h"
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     （灯）找上下左右边界
//-------------------------------------------------------------------------------------------------------------------
// 寻找上边界
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
// 寻找下边界
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
// 寻找左边界
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
// 寻找右边界
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
// 寻找内边界函数
InnerEdgeResult Find_Inner_Edges_Optimized(int top, int bottom, int left, int right)
{
    InnerEdgeResult result;
#if 0 == Find_Mode
    // 为加快照灯速率，该模式下不使用上下内边缘检测，初始化为 0 即可
    result.inner_top = 0;
    result.inner_bottom = 0;
    // 检测左右边缘
    int center = (left + right) / 2;

    // 扫不到线就给我滚到中值去
    result.inner_left = center;
    result.inner_right = center;

    // 向左扫描找内左边界
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
    // 为加快照灯速率，该模式下不使用左右内边缘检测，初始化为 0 即可
    result.inner_left = 0;
    result.inner_right = 0;
    // 检测上下外边缘
    int center = (top + bottom) / 2;

    // 扫不到线就给我滚到中值去
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
//  函数简介     中心线计算增强
//-------------------------------------------------------------------------------------------------------------------
float Calculate_Enhanced_CenterLine(int left, int inner_left, int right, int inner_right)
{
    // 原始边界中心计算
    float outer_center = ((float)(left + right)) / 2.0f;
#if 0 == Find_Mode
    // 内边界中心计算
    float inner_center = ((float)(inner_left + inner_right)) / 2.0f;
    // 动态权重：如果内外边界差异大（急转弯），降低内边界权重
    float boundary_diff = fabs(inner_left - inner_right);
    float inner_weight = (boundary_diff > 30.0f) ? 0.1f : 0.3f; // 差异大时权重降至10%
    float weighted_center = outer_center * (1.0f - inner_weight) + inner_center * inner_weight;
#else
    float weighted_center = outer_center;
#endif
    // 转换为角度（0-180°范围）并反转方向
    float CenterLine_Angle = 180.0f - (weighted_center / 128.0f * 180.0f);

    return CenterLine_Angle;
}
//-------------------------------------------------------------------------------------------------------------------
//  操作简介     卡尔曼滤波结构体全局实例化
//-------------------------------------------------------------------------------------------------------------------
KalmanFilter kf_centerline;
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     卡尔曼滤波器初始化
//-------------------------------------------------------------------------------------------------------------------
// 初始化卡尔曼滤波器（在程序初始化时调用）
void Kalman_Init(void)
{
    kf_centerline.q = 0.5f;     // 过程噪声（越小越信任预测）使滤波更依赖历史预测（滞后）
    kf_centerline.r = 0.01f;    // 观测噪声（越小越信任测量）
    kf_centerline.p = 1.0f;     // 初始估计误差
    kf_centerline.x = 90.0f;    // 初始角度（图像正中）
    kf_centerline.last_angle = 90.0f;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     卡尔曼滤波器计算
//-------------------------------------------------------------------------------------------------------------------
float Kalman_Update(float measurement)
{
    // 预测阶段
    kf_centerline.p = kf_centerline.p + kf_centerline.q;

    // 更新阶段
    kf_centerline.k = kf_centerline.p / (kf_centerline.p + kf_centerline.r);
    kf_centerline.x = kf_centerline.x + kf_centerline.k * (measurement - kf_centerline.x);
    kf_centerline.p = (1 - kf_centerline.k) * kf_centerline.p;

    return kf_centerline.x;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     车辆运行合计算
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
        // 加权抗噪
        float enhanced_centerline = Calculate_Enhanced_CenterLine(left, new_left, right, new_right);
        // 使用卡尔曼滤波（优先使用增强中心线）
        float filtered_angle = Kalman_Update(enhanced_centerline);

        // 将当前角度存入缓冲区
        angle_buffer[buffer_index] = filtered_angle;
        buffer_index = (buffer_index + 1) % DELAY_BUFFER_SIZE;
        // 使用缓冲区中最旧的角度（延迟转弯）
        CenterLine_Average = angle_buffer[buffer_index];
    }
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     画出灯的边界
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
//  函数简介     速度决策 与 发车
//-------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     PID 控制器初始化
//  参数说明     kp: 比例系数, kd: 微分系数，kd 使用的是转向的
//  返回参数     初始化好的 PID 控制器
//-------------------------------------------------------------------------------------------------------------------
PID_Controller PID_Init(float kp, float kd) {
    PID_Controller pid;
    pid.Kp = kp;
    pid.Kd = kd;
    pid.last_error = 0;
    return pid;
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介     计算 PID 输出
//  参数说明     pid: PID 控制器, error: 当前误差（范围：0-63）
//  返回参数     PID 输出值（范围：0-Max_Control），值越大表示速度越慢，越小速度越快
//-------------------------------------------------------------------------------------------------------------------
float PID_Compute(PID_Controller* pid, float error) {
    // PD 控制公式：输出 = Kp * error + Kd * derivative
    float output = pid->Kp * error + pid->Kd * (error - pid->last_error);
    pid->last_error = error;  // 更新上一次误差

    // 限制输出值在 0 到 Max_Control 之间
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
    // 发车
    if (Center != 127) {
        Stop_flag = false;
    }
    PID_Controller pid = PID_Init(Kp_Speed, Servo_pid_t.Dir_Kd);                //控距环pid
#if 0 == Find_Mode
    float control_signal = PID_Compute(&pid, Center);
#else
    int Center_High = bottom - top;
    float control_signal = PID_Compute(&pid, Center_High);
#endif
    Speed_Float = -control_signal;
}
//-------------------------------------------------------------------------------------------------------------------
//  函数简介     最终处理
//-------------------------------------------------------------------------------------------------------------------
void ImageProcess(void)
{
    // 获取边界
    int top = Find_Top_Edge();
    int bottom = Find_Bottom_Edge();
    int left = Find_Left_Edge();
    int right = Find_Right_Edge();
    // 中心扫边界
    InnerEdgeResult inner_edges = Find_Inner_Edges_Optimized(top, bottom, left, right);
    // 获取内边界值
    int new_left = inner_edges.inner_left;
    int new_right = inner_edges.inner_right;
#if 1 == Find_Mode
    int new_top = inner_edges.inner_top;
    int new_bottom = inner_edges.inner_bottom;
#endif
    // 车辆运行
    CarRun(top, bottom, left, new_left, right, new_right);
    // 速度决策与发车
    SpeedControl(top, bottom, left, right);
    // DEBUG: 画出边界
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
