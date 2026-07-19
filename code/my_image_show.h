#ifndef _my_image_show_h_
#define _my_image_show_h_

#include "zf_common_headfile.h"

//-------------------------------------------------------------------------------------------------------------------
// image_handle — ISR 内调用（TIM6 80Hz）：图像拷贝 + 大津法 + 巡线流水线
// image_show   — 主循环调用：仅 IPS200 显示（上半屏灰度 + 下半屏二值化 + 巡线）
//
// 调用后，巡线结果可通过 my_line_follow.h 中的全局变量获取：
//   Left[], Right[], Mid[]  — 边线/中线数据
//   Dir_err                  — 方向控制误差
//   imgTop                   — 截止行
//-------------------------------------------------------------------------------------------------------------------
extern void image_handle (void);
extern void image_show   (void);

#endif
