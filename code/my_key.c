#include "my_key.h"
#include "zf_components_menu.h"
#include "zf_device_key.h"

/* ---- 初始化 ---- */
void my_key_init(void)
{
    key_init(10); /* 10ms 扫描周期 */
}

/* ---- 主循环调用：按键扫描 + 动作映射 ---- */
void my_key_process(void)
{
    key_scanner();

    switch(menu_get_state())
    {
    /* ===== 菜单关闭 ===== */
    case MENU_CLOSED:
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)     /* E2 */
        {
            menu_open();
            key_clear_state(KEY_1);
        }
        break;

    /* ===== 浏览模式 ===== */
    case MENU_BROWSING:
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)     /* E2: 进文件夹/编参 */
        {
            menu_open();    /* menu_open 内部在 BROWSING 态调用 menu_enter */
            key_clear_state(KEY_1);
        }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)     /* E3: 返回/退出 */
        {
            menu_back();
            key_clear_state(KEY_2);
        }
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)     /* E4: 下一项 */
        {
            menu_next();
            key_clear_state(KEY_3);
        }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)     /* E5: 选中编辑 */
        {
            menu_select();
            key_clear_state(KEY_4);
        }
        break;

    /* ===== 编辑模式 ===== */
    case MENU_EDITING:
        if(key_get_state(KEY_1) == KEY_SHORT_PRESS)     /* E2: 减小 */
        {
            menu_open();    /* menu_open 内部在 EDITING 态调用 menu_dec */
            key_clear_state(KEY_1);
        }
        if(key_get_state(KEY_2) == KEY_SHORT_PRESS)     /* E3: 增大 */
        {
            menu_back();    /* menu_back 内部在 EDITING 态调用 menu_inc */
            key_clear_state(KEY_2);
        }
        if(key_get_state(KEY_3) == KEY_SHORT_PRESS)     /* E4: 确认→下一项 */
        {
            menu_next();
            key_clear_state(KEY_3);
        }
        if(key_get_state(KEY_4) == KEY_SHORT_PRESS)     /* E5: 取消编辑 */
        {
            menu_select();
            key_clear_state(KEY_4);
        }
        break;
    }
}
