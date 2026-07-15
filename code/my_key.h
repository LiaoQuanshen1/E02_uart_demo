#ifndef _my_key_h_
#define _my_key_h_

#include "zf_common_headfile.h"

void my_key_init(void);       /* 初始化按键模块       */
void my_key_process(void);    /* 主循环调用：扫描+映射 */

#endif
