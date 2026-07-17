#ifndef _my_line_follow_h_
#define _my_line_follow_h_

#include "zf_common_headfile.h"

// ============================================================
// 图像尺寸（与 MT9V03X 摄像头配置保持一致）
// ============================================================
#define LINE_IMG_W    MT9V03X_W    // 188 列
#define LINE_IMG_H    MT9V03X_H    // 120 行

// ============================================================
// 区域阈值补偿（ApplyEdgeCompensation）
// 目的：镜头径向畸变导致边缘偏暗，对左右边缘区域降低二值化阈值
// ============================================================
#define EDGE_COMP_THRESHOLD_MIN     70    // Otsu 阈值下限（低于此值强制设为 70）
#define EDGE_COMP_LEFT_BOUNDARY     36    // 左边缘补偿列范围 [0, 36]
#define EDGE_COMP_RIGHT_LOW         152   // 右边缘补偿起始列（col >= 152）
#define EDGE_COMP_THRESHOLD_DELTA   10    // 边缘区域阈值降低量

// ============================================================
// 边线搜索（FindSidelines）
// ============================================================
#define SIDELINE_TOLERANCE_COL      6     // "白-黑-白" 容错模式：距上一行边线最大列偏差

// ============================================================
// 动态前瞻行（CalculateError）
// forward = FORWARD_DEFAULT - (当前速度 / FORWARD_SPEED_DIVISOR)
// 速度越快 forward 越小 → 看得越远
// ============================================================
#define FORWARD_DEFAULT             60    // 默认前瞻行号
#define FORWARD_MAX                 100   // 前瞻行号上限
#define FORWARD_SPEED_DIVISOR       30    // 速度除数

// ============================================================
// 误差计算（CalculateError）
// Dir_err > 0 → 中线偏左 → 车应左转
// Dir_err < 0 → 中线偏右 → 车应右转
// ============================================================
#define DIR_ERR_MAX                 94    // Dir_err 绝对值上限（≈半宽 LINE_IMG_W/2）
#define DIR_ERR_DELTA_MAX           8.0f  // 帧间误差变化率上限（抗突变）

// ============================================================
// 拐点检测（FindGuaidians）
// ============================================================
#define GUAI_WIDTH_INCREASE_UP      10    // 上拐点：赛道宽度增加阈值（列）
#define GUAI_WIDTH_INCREASE_DOWN    20    // 下拐点：赛道宽度减小阈值（列）
#define GUAI_SLOPE_LIMIT            1.0f  // 拐点边线斜率限幅

// ============================================================
// 补线（Buxian）底部参考点参数
// ============================================================
#define BUXIAN_BOTTOM_ROW_OFFSET    5     // 底部参考点距离底边的行偏移
#define BUXIAN_BOTTOM_COL_OFFSET    6     // 底部参考点距离边线的列偏移

// ============================================================
// 功能开关
// ============================================================
#define ENABLE_GUAI_DETECTION       1     // 1=启用拐点检测与补线，0=仅基本巡线

// ============================================================
// 拐点结构体
// ============================================================
typedef struct {
    int  row;      // 拐点所在行号
    int  col;      // 拐点所在列号
    bool found;    // 是否检测到
} GuaiPoint;

// ============================================================
// 全局状态 — 供控制算法和屏幕绘制引用
// ============================================================
// 注意：数组大小为 LINE_IMG_H+1，多出 1 个哨兵元素避免 FindSidelines 访问 row+1 越界
// 外部代码只应访问索引 [0, LINE_IMG_H-1]
extern int   Left[LINE_IMG_H + 1];       // 左边线列号，Left[i]=第i行左边界列
extern int   Right[LINE_IMG_H + 1];      // 右边线列号，Right[i]=第i行右边界列
extern int   Mid[LINE_IMG_H + 1];        // 中线列号，Mid[i]=(Left[i]+Right[i])/2
extern int   WhiteWidth[LINE_IMG_H + 1]; // 每行赛道宽度（列）
extern int   imgTop;                    // 截止行（赛道最远可见行，0=最远，LINE_IMG_H-1=最近）
extern int   maxColumn;                 // 最长白列的列号（边线搜索起始列）
extern float Dir_err;                   // 方向控制误差（正值=左转，负值=右转）
extern int   L_loseCount;               // 左侧丢线总行数
extern int   R_loseCount;               // 右侧丢线总行数

// ============================================================
// 二值化结果数组 — 供屏幕显示使用
// 255 = 白色（赛道），0 = 黑色（背景/边线外）
// ============================================================
extern uint8 line_binary[LINE_IMG_H][LINE_IMG_W];

// ============================================================
// 公开 API
// ============================================================

// 每帧调用一次的处理入口。
// otsu_threshold: 大津法计算出的全局最优阈值
// gray_img:       原始灰度图像（不会被修改）
void ProcessFrame(uint8 otsu_threshold, const uint8 gray_img[LINE_IMG_H][LINE_IMG_W]);

#endif
