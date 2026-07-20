#ifndef _my_image_show_h_
#define _my_image_show_h_
#define IMAGE_SHOW_UART
#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// image_handle — ISR 内调用（TIM6 80Hz）：图像拷贝 + 大津法 + 巡线流水线
// image_show   — 主循环调用：图像显示
//                 若 #define IMAGE_SHOW_UART → 通过 DEBUG 串口发送至逐飞助手上位机
//                 否则 → IPS200 屏幕显示（上半屏灰度 + 下半屏二值化 + 巡线）
//
// 上位机显示内容（与 IPS200 屏幕显示一致）：
//   - 灰度图像 + 3 条彩色边线（左边界/中线/右边界）
//   - 拐点：7×7 白色方块标记（与屏幕 draw_marker 同形状）
//   - 最远线（截止行）：白色水平线
//
// 调用后，巡线结果可通过 my_line_follow.h 中的全局变量获取：
//   Left[], Right[], Mid[]  — 边线/中线数据
//   Dir_err                  — 方向控制误差
//   imgTop                   — 截止行
//-------------------------------------------------------------------------------------------------------------------
extern void image_handle (void);
extern void image_show   (void);

#endif
