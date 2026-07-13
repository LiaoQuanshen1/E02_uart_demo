#include "my_image_show.h"

static uint8  image_buf[MT9V03X_H][MT9V03X_W];                                 // 本地图像缓冲，防止 DMA 覆盖

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

uint16 left_edge[MT9V03X_H];                                                   // 每行左边界 x 坐标
uint16 right_edge[MT9V03X_H];                                                  // 每行右边界 x 坐标
uint16 center_line[MT9V03X_H];                                                 // 每行中线   x 坐标
uint8  edge_valid[MT9V03X_H];                                                  // 每行巡线是否有效

//-------------------------------------------------------------------------------------------------------------------
// 巡线：从底部中间向两侧扫描，白-白-黑模式识别边界，中线迭代传递到上一行
// 赛道白色（≥thresh），背景黑色（<thresh）
//-------------------------------------------------------------------------------------------------------------------
static void find_lines (uint8 thresh)
{
    for(uint16 i = 0; i < MT9V03X_H; i++)                                      // 全部初始化为无效
    {
        left_edge[i]  = 0xFFFF;
        right_edge[i] = 0xFFFF;
        edge_valid[i] = 0;
    }

    uint16 start_x = MT9V03X_W / 2;                                            // 底部起始：图像中间列

    for(int16 r = MT9V03X_H - 1; r >= 0; r--)                                  // 从底部向上逐行扫描
    {
        if(image_buf[r][start_x] < thresh) break;                              // 起点不在赛道上，停止处理

        uint16 left  = 0xFFFF;
        uint16 right = 0xFFFF;

        // 向左扫描：找 白-白-黑 组合 → 左边界为倒数第二个白点
        for(int16 c = start_x; c >= 2; c--)
        {
            if(image_buf[r][c] >= thresh
                    && image_buf[r][c - 1] >= thresh
                    && image_buf[r][c - 2] < thresh)
            {
                left = (uint16)(c - 1);
                break;
            }
        }
        if(left == 0xFFFF) left = 0;                                            // 赛道延伸到图像左边缘

        // 向右扫描：找 白-白-黑 组合 → 右边界为倒数第二个白点
        for(uint16 c = start_x; c <= MT9V03X_W - 3; c++)
        {
            if(image_buf[r][c] >= thresh
                    && image_buf[r][c + 1] >= thresh
                    && image_buf[r][c + 2] < thresh)
            {
                right = (uint16)(c + 1);
                break;
            }
        }
        if(right == 0xFFFF) right = MT9V03X_W - 1;                              // 赛道延伸到图像右边缘

        // 记录结果，中线作为上一行的扫描起点
        if(left < right)
        {
            left_edge[r]   = left;
            right_edge[r]  = right;
            center_line[r] = (left + right) >> 1;
            edge_valid[r]  = 1;
        }
        // 无论如何都更新 start_x，避免旧中线在新行误触 break
        // 若本行无效，用上一次有效的中线继续向上传播
        if(edge_valid[r])
            start_x = center_line[r];
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 在 IPS200 上绘制巡线结果（逐点绘制）
//-------------------------------------------------------------------------------------------------------------------
static void draw_lines (void)
{
    for(uint16 r = 0; r < MT9V03X_H; r++)
    {
        if(!edge_valid[r]) continue;

        ips200_draw_point(left_edge[r],   r, RGB565_GREEN);                     // 左边界 — 绿色
        ips200_draw_point(right_edge[r],  r, RGB565_BLUE);                     // 右边界 — 蓝色
        ips200_draw_point(center_line[r], r, RGB565_RED);                      // 中线   — 红色
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 图像采集 + 大津法二值化 + 显示 + 巡线绘制
//-------------------------------------------------------------------------------------------------------------------
void image_show (void)
{
    if(!mt9v03x_finish_flag) return;

    // ① 拷贝到本地缓冲，防止 DMA 在新一帧写入时覆盖数据
    memcpy(image_buf, mt9v03x_image, MT9V03X_IMAGE_SIZE);
    mt9v03x_finish_flag = 0;                                                   // 提前释放标志，减少丢帧

    // ② 大津法求阈值
    uint8 thresh = otsu_threshold(image_buf);

    // ③ 显示二值化图像（必须先于画线，避免线条被覆盖）
    ips200_clear();
    ips200_show_gray_image(0, 0, (uint8 *)image_buf, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, thresh);

    // ④ 巡线 + 绘制边界和中线
    find_lines(thresh);
    draw_lines();
}


