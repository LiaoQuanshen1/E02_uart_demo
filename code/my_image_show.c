#include "my_image_show.h"
#include "my_line_follow.h"
#include "zf_driver_dma.h"

static uint8  image_buf[MT9V03X_H][MT9V03X_W];                                 // 本地图像缓冲，防止 DMA 覆盖
static vuint8 frame_processed_flag = 0;                                         // 一帧图像处理完成标志（ISR→主循环）

//-------------------------------------------------------------------------------------------------------------------
// 大津法（Otsu）计算最优二值化阈值
//-------------------------------------------------------------------------------------------------------------------
static uint8 otsu_threshold (uint8 img[MT9V03X_H][MT9V03X_W])
{
    static uint32 hist[256];                                                    // static 避免栈溢出（1KB）
    memset(hist, 0, sizeof(hist));

    uint32 total = MT9V03X_IMAGE_SIZE;
    for(uint16 r = 0; r < MT9V03X_H; r++)                                      // 统计灰度直方图
        for(uint16 c = 0; c < MT9V03X_W; c++)
            hist[img[r][c]]++;

    float  max_var = 0;
    uint8  best_t  = 128;
    uint32 sum_all = 0;

    for(uint16 i = 0; i < 256; i++)
        sum_all += i * hist[i];

    uint32 w1 = 0, sum1 = 0;
    for(uint16 t = 0; t < 256; t++)
    {
        w1   += hist[t];
        sum1 += t * hist[t];
        if(w1 == 0 || w1 == total) continue;

        uint32 w2   = total - w1;
        uint32 sum2 = sum_all - sum1;
        float  m1   = (float)sum1 / w1;
        float  m2   = (float)sum2 / w2;
        float  var  = (float)w1 * w2 * (m1 - m2) * (m1 - m2);

        if(var > max_var)
        {
            max_var = var;
            best_t  = (uint8)t;
        }
    }
    return best_t;
}

//-------------------------------------------------------------------------------------------------------------------
// 在 IPS200 上绘制巡线结果（逐点绘制，y 偏移至下半屏）
// 数据来源：my_line_follow 模块的 Left/Right/Mid 全局数组
//-------------------------------------------------------------------------------------------------------------------
#define DRAW_OFFSET_Y   (MT9V03X_H)                                             // 巡线绘制在二值化图像区域，y 偏移一个图像高度

// IPS200 竖屏分辨率（驱动内部 ips200_x_max=240, ips200_y_max=320）
#define MARKER_X_MAX    240
#define MARKER_Y_MAX    320

// 在 (x,y) 处绘制一个小方块标记（带屏幕边界裁剪）
static void draw_marker(uint16 x, uint16 y, uint16 color)
{
    int sx = (int)x, sy = (int)y;
    for (int i = -3; i <= 3; i++) {
        int px = sx + i;
        if (px < 0 || px >= MARKER_X_MAX) continue;
        for (int j = -3; j <= 3; j++) {
            int py = sy + j;
            if (py < 0 || py >= MARKER_Y_MAX) continue;
            ips200_draw_point((uint16)px, (uint16)py, color);
        }
    }
}

static void draw_lines (void)
{
    uint16 dy;

    // --- 绘制截止行（最远可见行）---
    dy = (uint16)imgTop + DRAW_OFFSET_Y;
    ips200_draw_line(0, dy, LINE_IMG_W - 1, dy, RGB565_CYAN);

    // --- 绘制边线与中线 ---
    for (int r = imgTop + 1; r < LINE_IMG_H; r++)
    {
        dy = (uint16)r + DRAW_OFFSET_Y;
        ips200_draw_point((uint16)Left[r],  dy, RGB565_BLUE);                 // 左边界 — 蓝色
        ips200_draw_point((uint16)Right[r], dy, RGB565_BLUE);                 // 右边界 — 蓝色
        ips200_draw_point((uint16)Mid[r],   dy, RGB565_RED);                  // 中线   — 红色
    }

    // --- 绘制拐点 ---
    if (L_h.found) draw_marker((uint16)L_h.col, (uint16)L_h.row + DRAW_OFFSET_Y, RGB565_YELLOW);
    if (L_l.found) draw_marker((uint16)L_l.col, (uint16)L_l.row + DRAW_OFFSET_Y, RGB565_GREEN);
    if (R_h.found) draw_marker((uint16)R_h.col, (uint16)R_h.row + DRAW_OFFSET_Y, RGB565_PURPLE);
    if (R_l.found) draw_marker((uint16)R_l.col, (uint16)R_l.row + DRAW_OFFSET_Y, RGB565_CYAN);
}

//-------------------------------------------------------------------------------------------------------------------
// image_handle — ISR 内调用（TIM6 80Hz）：图像拷贝 + Otsu + 巡线流水线
// 产出：Dir_err（供 control_run 消费）、line_binary（供显示消费）
//-------------------------------------------------------------------------------------------------------------------
void image_handle(void)
{
    if(!mt9v03x_finish_flag) return;

    // ① 关 DMA → 原子拷贝 → 释标志
    dma_disable(MT9V03X_DMA_CH);
    memcpy(image_buf, mt9v03x_image, MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 0;

    // ② 大津法求阈值
    uint8 thresh = otsu_threshold(image_buf);

    // ③ 巡线流水线 → 更新 Dir_err / Left / Right / Mid 等全局变量
    ProcessFrame(thresh, image_buf);

    // ④ 通知主循环：新一帧已就绪，可以刷新显示
    frame_processed_flag = 1;
}

//-------------------------------------------------------------------------------------------------------------------
// image_show — 主循环调用：仅 IPS200 显示（无图像处理）
// 布局：上半屏 = 原始灰度，下半屏 = 二值化图像 + 巡线
//-------------------------------------------------------------------------------------------------------------------
void image_show (void)
{
    if(!frame_processed_flag) return;

    // ① 上半屏：原始灰度
    ips200_show_gray_image(0, 0, (uint8 *)image_buf, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);

    // ② 下半屏：二值化图像
    ips200_show_gray_image(0, MT9V03X_H, (uint8 *)line_binary, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 128);

    // ③ 绘制边界和中线
    draw_lines();

    frame_processed_flag = 0;
}
