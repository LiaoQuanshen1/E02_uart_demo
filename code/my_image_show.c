#include "my_image_show.h"
#include "my_line_follow.h"
#include "zf_driver_dma.h"

#ifdef IMAGE_SHOW_UART
#include "seekfree_assistant.h"
#include "seekfree_assistant_interface.h"

//-------------------------------------------------------------------------------------------------------------------
// 串口发送回调 — 适配 seekfree_assistant 的 transfer 接口，通过 DEBUG 串口发送
//-------------------------------------------------------------------------------------------------------------------
static uint32 image_uart_send (const uint8 *buff, uint32 len)
{
    uart_write_buffer(DEBUG_UART_INDEX, buff, len);
    return 0;                                                                   // 阻塞式发送，全部完成
}

// 边线数据 uint8 副本 — seekfree_assistant X_BOUNDARY 协议要求 8bit 坐标（值域 0~187 不溢出）
static uint8 left_u8[LINE_IMG_H];
static uint8 mid_u8[LINE_IMG_H];
static uint8 right_u8[LINE_IMG_H];

// 二值图位压缩缓冲区：188×120 / 8 = 2820 字节
static uint8 binary_packed[MT9V03X_W * MT9V03X_H / 8];

//-------------------------------------------------------------------------------------------------------------------
// 将 line_binary（每像素 0/255）压缩为位图（每字节 8 像素，MSB 先行）
// 复用现有的 line_binary 数组，与 IPS200 下半屏显示的二值化图像同源
//-------------------------------------------------------------------------------------------------------------------
static void pack_binary (uint8 *dst, const uint8 src[LINE_IMG_H][LINE_IMG_W])
{
    uint16 byte_idx  = 0;
    uint8  bit_shift = 0;                                                       // 0=MSB … 7=LSB
    memset(dst, 0, MT9V03X_W * MT9V03X_H / 8);

    for (int r = 0; r < LINE_IMG_H; r++)
    {
        for (int c = 0; c < LINE_IMG_W; c++)
        {
            if (src[r][c])                                                      // 白色(255) → 置位
                dst[byte_idx] |= (0x80 >> bit_shift);
            if (++bit_shift == 8)
            {
                bit_shift = 0;
                byte_idx++;
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 在二值图 line_binary 上叠加拐点标记与最远线（与 IPS200 draw_marker 形状一致）
// line_binary 在下一次 ProcessFrame 调用时被完整重写，标记无需回退
//-------------------------------------------------------------------------------------------------------------------
static void mark_overlays_binary (uint8 img[LINE_IMG_H][LINE_IMG_W])
{
    // --- 最远线（截止行），对应 IPS200: ips200_draw_line(..., RGB565_CYAN) ---
    if (imgTop >= 0 && imgTop < LINE_IMG_H)
        memset(img[imgTop], 0xFF, LINE_IMG_W);

    // --- 拐点 7×7 方块，对应 IPS200: draw_marker(±3 像素方块) ---
    #define STAMP(r, c) do {                                                    \
        int _r = (r), _c = (c);                                                 \
        for (int di = -3; di <= 3; di++)                                        \
            for (int dj = -3; dj <= 3; dj++) {                                  \
                int pr = _r + di, pc = _c + dj;                                  \
                if (pr >= 0 && pr < LINE_IMG_H && pc >= 0 && pc < LINE_IMG_W)   \
                    img[pr][pc] = 0xFF;                                          \
            }                                                                   \
    } while(0)

    if (L_h.found) STAMP(L_h.row, L_h.col);     // 左上拐点
    if (L_l.found) STAMP(L_l.row, L_l.col);     // 左下拐点
    if (R_h.found) STAMP(R_h.row, R_h.col);     // 右上拐点
    if (R_l.found) STAMP(R_l.row, R_l.col);     // 右下拐点
    #undef STAMP
}
#endif

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

#ifndef IMAGE_SHOW_UART  // === IPS200 显示专用函数（UART 模式不需要）===

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

#endif  // !IMAGE_SHOW_UART

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
// image_show — 主循环调用：图像显示
//   - 若 IMAGE_SHOW_UART 已定义 → 通过 DEBUG 串口发送二值化图像+彩色边线至逐飞助手上位机
//   - 否则 → IPS200 屏幕显示（上半屏灰度 + 下半屏二值化 + 巡线）
//-------------------------------------------------------------------------------------------------------------------
void image_show (void)
{
    if(!frame_processed_flag) return;

#ifdef IMAGE_SHOW_UART
    // ========== UART 上位机显示模式（通过 DEBUG 串口，发送二值化图像）==========
    // ① 首次调用：挂载串口发送回调 + 配置图像信息（二值图类型，复用 line_binary）
    static bool uart_inited = false;
    if(!uart_inited)
    {
        extern seekfree_assistant_transfer_callback_function
            seekfree_assistant_transfer_callback;
        seekfree_assistant_transfer_callback = image_uart_send;
        seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_CUSTOM);
        // 使用 OV7725_BIN（= BINARY）类型，image 指向位压缩缓冲区
        seekfree_assistant_camera_information_config(
            SEEKFREE_ASSISTANT_OV7725_BIN, binary_packed, MT9V03X_W, MT9V03X_H);
        uart_inited = true;
    }

    // ② 在二值图 line_binary 上叠加标记（与 IPS200 下半屏显示一致）：
    //    最远线 → 白色水平线  拐点 → 7×7 白色方块
    //    line_binary 在下次 ProcessFrame 时被完整重写，无需回退
    mark_overlays_binary(line_binary);

    // ③ 将标记后的二值图位压缩 → binary_packed（每字节 8 像素，MSB 先行）
    pack_binary(binary_packed, line_binary);

    // ④ 将 int 边线数组转为 uint8（上位机 X_BOUNDARY 协议使用 8bit 坐标）
    for (int r = 0; r < LINE_IMG_H; r++)
    {
        left_u8[r]  = (uint8)Left[r];
        mid_u8[r]   = (uint8)Mid[r];
        right_u8[r] = (uint8)Right[r];
    }

    // ⑤ 配置边线 + 发送（二值图 ~2820 字节 + 边界 ~360 字节 ≈ 3.2KB，~278ms@115200）
    seekfree_assistant_camera_boundary_config(
        X_BOUNDARY, LINE_IMG_H,
        left_u8, mid_u8, right_u8,
        NULL, NULL, NULL);
    seekfree_assistant_camera_send();

#else
    // ========== IPS200 屏幕显示模式（原逻辑）==========
    // ① 上半屏：原始灰度
    ips200_show_gray_image(0, 0, (uint8 *)image_buf, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);

    // ② 下半屏：二值化图像
    ips200_show_gray_image(0, MT9V03X_H, (uint8 *)line_binary, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 128);

    // ③ 绘制边界和中线
    draw_lines();

#endif

    frame_processed_flag = 0;
}
