#include "zf_components_menu.h"
#include "zf_device_ips200.h"
#include "zf_device_key.h"
#include "string.h"

/* ==================== 内部静态变量 ==================== */
static MENU_ITEM head;                  /* 菜单根节点                         */
static MENU_ITEM *key;                 /* 当前光标所在节点                   */
static menu_mode_t menu_mode = MENU_MODE_NAVIGATE;
static uint8_t scroll_offset = 0;      /* 当前显示窗口的滚动偏移（第几个子项）*/
static bool menu_visible = false;      /* 菜单是否可见                       */

/* 参数描述符静态池 */
static param_desc_t param_pool[MENU_MAX_PARAM_POOL];
static uint8_t param_pool_used = 0;

/* ==================== 参数描述符池管理 ==================== */
static param_desc_t *param_alloc(void)
{
    if (param_pool_used < MENU_MAX_PARAM_POOL)
        return &param_pool[param_pool_used++];
    return NULL;  /* 池已满 */
}

/* ==================== 基础菜单项创建 ==================== */
static void Create_Menu_Item(MENU_ITEM *father, MENU_ITEM *me, const char name[],
                             void *data, MENU_KIND kind)
{
    if (father->kind != MENU_FOLDER)
        return;  /* 只有文件夹下面才可以创建成员 */

    me->name         = name;
    me->sons         = 0;
    me->select       = false;
    me->data         = data;
    me->kind         = kind;
    me->father       = father;
    me->first_child  = NULL;
    me->prev_brother = NULL;
    me->next_brother = NULL;

    if (father->sons == 0)
    {
        father->first_child = me;
    }
    else
    {
        MENU_ITEM *p = father->first_child;
        while (p->next_brother != NULL && p->next_brother != father->first_child)
            p = p->next_brother;  /* 找到同一层级的最后节点 */
        p->next_brother = me;
        me->prev_brother = p;     /* 建立同一层级的连接 */
    }
    /* 建立环形双向链表，防止指针跑飞 */
    me->next_brother = father->first_child;
    father->first_child->prev_brother = me;

    me->index = father->sons;
    father->sons++;
}

/* ==================== 创建文件夹 ==================== */
void Create_Menu_Folder(MENU_ITEM *father, MENU_ITEM *me, const char name[])
{
    Create_Menu_Item(father, me, name, NULL, MENU_FOLDER);
}

/* ==================== 创建数值型菜单项（含步长/最小/最大） ==================== */
void Create_Menu_Number(MENU_ITEM *father, MENU_ITEM *me, const char name[],
                        void *value_ptr, MENU_KIND kind,
                        float step, float min_val, float max_val)
{
    param_desc_t *p = param_alloc();
    if (p == NULL) return;  /* 池已满 */

    p->value_ptr = value_ptr;
    p->kind      = kind;
    p->step      = step;
    p->min_val   = min_val;
    p->max_val   = max_val;

    /* 如果是 bool 类型，初始化为 false */
    if (kind == bool_Box)
        *(bool *)value_ptr = false;

    Create_Menu_Item(father, me, name, (void *)p, kind);
}

/* ==================== 菜单初始化 ==================== */
void menu_init(void)
{
    /* 初始化参数池 */
    param_pool_used = 0;
    memset(param_pool, 0, sizeof(param_pool));

    /* 初始化根节点 */
    head.data         = NULL;
    head.father       = NULL;
    head.first_child  = NULL;
    head.kind         = MENU_FOLDER;
    head.name         = "MENU";
    head.next_brother = NULL;
    head.prev_brother = NULL;
    head.sons         = 0;
    head.index        = 0;

    key          = &head;
    menu_mode    = MENU_MODE_NAVIGATE;
    scroll_offset = 0;
    menu_visible  = false;
}

/* ==================== 获取菜单模式 ==================== */
menu_mode_t menu_get_mode(void)
{
    return menu_mode;
}

/* ==================== 数值显示辅助函数 ==================== */
static void show_param_value(uint16 x, uint16 y, param_desc_t *p, uint16 color)
{
    char buf[16];
    switch (p->kind)
    {
    case int_Box:
        ips200_show_int(x, y, *(int *)p->value_ptr, 6);
        break;
    case float_Box:
        ips200_show_float(x, y, *(float *)p->value_ptr, 7, 1);
        break;
    case bool_Box:
        ips200_show_string(x, y, *(bool *)p->value_ptr ? "ON " : "OFF");
        break;
    case uint8_Box:
        ips200_show_uint(x, y, *(uint8_t *)p->value_ptr, 3);
        break;
    case uint16_Box:
        ips200_show_uint(x, y, *(uint16_t *)p->value_ptr, 5);
        break;
    case uint32_Box:
        ips200_show_uint(x, y, *(uint32_t *)p->value_ptr, 8);
        break;
    default:
        break;
    }
}

/* ==================== IPS200 绘制菜单界面 ==================== */
void menu_show(void)
{
    if (!menu_visible) return;

    MENU_ITEM *h = key->father;          /* 当前所在文件夹 */
    uint8_t total = h->sons;             /* 同级子项总数   */
    if (total == 0) return;              /* 空文件夹       */

    /* ---- 计算可见窗口 ---- */
    uint8_t visible = MENU_MAX_VISIBLE;
    if (total < visible) visible = total;

    /* 调整 scroll_offset，保证 key 在可见窗口内 */
    if (key->index < scroll_offset)
        scroll_offset = key->index;
    else if (key->index >= scroll_offset + visible)
        scroll_offset = key->index - visible + 1;

    /* ---- 绘制标题栏 ---- */
    ips200_show_string(0, MENU_DISPLAY_Y, (char *)h->name);

    /* 如果有上级，显示 ".." 返回提示 */
    if (h->father != NULL)
        ips200_show_string(90, MENU_DISPLAY_Y, " [BACK]");

    /* ---- 绘制子项列表 ---- */
    MENU_ITEM *s = h->first_child;
    for (uint8_t i = 0; i < scroll_offset && i < total; i++)
        s = s->next_brother;  /* 跳到滚动起始位置 */

    for (uint8_t i = 0; i < visible; i++)
    {
        uint16 line_y = MENU_DISPLAY_Y + MENU_LINE_HEIGHT + i * MENU_LINE_HEIGHT;
        uint16 color  = (s == key) ? MENU_SELECT_COLOR : MENU_NORMAL_COLOR;

        /* 编辑模式下当前项用红色高亮 */
        if (menu_mode == MENU_MODE_EDIT && s == key)
            color = MENU_EDIT_COLOR;

        /* 光标指示符 */
        ips200_show_string(0, line_y, (s == key) ? ">" : " ");

        /* 项名称（截断到 10 字符以适应窄屏） */
        ips200_show_string(8, line_y, (char *)s->name);

        if (s->kind == MENU_FOLDER)
        {
            /* 文件夹：显示 '...' 或子项数提示 */
            ips200_show_string(80, line_y, "[>]");
        }
        else if (s->data != NULL)
        {
            /* 数值项：显示当前值 */
            param_desc_t *p = (param_desc_t *)s->data;
            ips200_show_string(72, line_y, ":");
            show_param_value(80, line_y, p, color);
        }

        s = s->next_brother;
    }

    /* ---- 滚动提示 ---- */
    if (scroll_offset > 0)
        ips200_show_string(120, MENU_DISPLAY_Y + MENU_LINE_HEIGHT, "..");
    if (scroll_offset + visible < total)
    {
        uint16 bottom_y = MENU_DISPLAY_Y + MENU_LINE_HEIGHT + visible * MENU_LINE_HEIGHT;
        ips200_show_string(120, bottom_y, "..");
    }

    /* ---- 编辑模式提示 ---- */
    if (menu_mode == MENU_MODE_EDIT)
        ips200_show_string(0, MENU_DISPLAY_Y + MENU_LINE_HEIGHT + (visible + 1) * MENU_LINE_HEIGHT,
                           "E2+ E3- E4 OK");
}

/* ==================== 参数增减 ==================== */
static void param_adjust(MENU_ITEM *item, int8_t direction)
{
    /* direction: +1 增大, -1 减小 */
    if (item == NULL || item->data == NULL) return;

    param_desc_t *p = (param_desc_t *)item->data;
    float step = p->step;

    /* bool 类型特殊处理：直接翻转 */
    if (p->kind == bool_Box)
    {
        *(bool *)p->value_ptr = !*(bool *)p->value_ptr;
        return;
    }

    switch (p->kind)
    {
    case float_Box:
    {
        float fv = *(float *)p->value_ptr + direction * step;
        if (fv > p->max_val) fv = p->max_val;
        if (fv < p->min_val) fv = p->min_val;
        *(float *)p->value_ptr = fv;
        break;
    }
    case int_Box:
    {
        float fv = (float)(*(int *)p->value_ptr) + direction * step;
        if (fv > p->max_val) fv = p->max_val;
        if (fv < p->min_val) fv = p->min_val;
        *(int *)p->value_ptr = (int)fv;
        break;
    }
    case uint8_Box:
    {
        float fv = (float)(*(uint8_t *)p->value_ptr) + direction * step;
        if (fv > p->max_val) fv = p->max_val;
        if (fv < p->min_val) fv = p->min_val;
        *(uint8_t *)p->value_ptr = (uint8_t)fv;
        break;
    }
    case uint16_Box:
    {
        float fv = (float)(*(uint16_t *)p->value_ptr) + direction * step;
        if (fv > p->max_val) fv = p->max_val;
        if (fv < p->min_val) fv = p->min_val;
        *(uint16_t *)p->value_ptr = (uint16_t)fv;
        break;
    }
    case uint32_Box:
    {
        float fv = (float)(*(uint32_t *)p->value_ptr) + direction * step;
        if (fv > p->max_val) fv = p->max_val;
        if (fv < p->min_val) fv = p->min_val;
        *(uint32_t *)p->value_ptr = (uint32_t)fv;
        break;
    }
    default:
        break;
    }
}

/* ==================== 清除菜单绘制区域 ==================== */
static void menu_clear_area(void)
{
    /* 用背景色填充菜单区域，避免残留 */
    for (uint16 y = MENU_DISPLAY_Y; y < 240; y++)
    {
        /* 逐行清除效率低但对 SPI 屏幕来说还能接受 */
        ips200_show_string(0, y, "                    ");  /* 20 个空格覆盖宽度 */
    }
}

/* ==================== 按键处理（在主循环中调用） ==================== */
void menu_key_handler(void)
{
    key_state_enum state;

    /* ---- E4 (ENTER) ---- */
    state = key_get_state(MENU_KEY_ENTER);
    if (state == KEY_SHORT_PRESS)
    {
        key_clear_state(MENU_KEY_ENTER);

        if (!menu_visible)
        {
            /* 菜单未显示 → 呼出菜单 */
            menu_visible = true;
            menu_mode = MENU_MODE_NAVIGATE;
            key = (head.first_child != NULL) ? head.first_child : &head;
            scroll_offset = 0;
            menu_clear_area();
            return;
        }

        if (menu_mode == MENU_MODE_EDIT)
        {
            /* 编辑模式 → 确认并退出编辑 */
            menu_mode = MENU_MODE_NAVIGATE;
            return;
        }

        /* 导航模式 → 进入文件夹或开始编辑 */
        if (key->kind == MENU_FOLDER && key->sons > 0)
        {
            key = key->first_child;
            scroll_offset = 0;
            menu_clear_area();
        }
        else if (key->kind != MENU_FOLDER && key->data != NULL)
        {
            menu_mode = MENU_MODE_EDIT;
        }
        return;
    }

    /* ---- E5 (BACK) ---- */
    state = key_get_state(MENU_KEY_BACK);
    if (state == KEY_SHORT_PRESS)
    {
        key_clear_state(MENU_KEY_BACK);

        if (!menu_visible) return;

        if (menu_mode == MENU_MODE_EDIT)
        {
            /* 编辑模式 → 退出编辑 */
            menu_mode = MENU_MODE_NAVIGATE;
            return;
        }

        /* 导航模式 → 返回上级或关闭菜单 */
        if (key->father != NULL && key->father->father != NULL)
        {
            /* 返回上级文件夹 */
            key = key->father;
            scroll_offset = 0;
            menu_clear_area();
        }
        else
        {
            /* 已在根目录 → 关闭菜单 */
            menu_visible = false;
            menu_clear_area();
        }
        return;
    }

    if (!menu_visible) return;  /* 菜单未显示时忽略上下键 */

    /* ---- E2 (UP) ---- */
    state = key_get_state(MENU_KEY_UP);
    if (state == KEY_SHORT_PRESS)
    {
        key_clear_state(MENU_KEY_UP);

        if (menu_mode == MENU_MODE_EDIT)
        {
            param_adjust(key, +1);  /* 增大值 */
        }
        else
        {
            key = key->prev_brother;  /* 上一个兄弟节点 */
        }
        return;
    }

    /* ---- E3 (DOWN) ---- */
    state = key_get_state(MENU_KEY_DOWN);
    if (state == KEY_SHORT_PRESS)
    {
        key_clear_state(MENU_KEY_DOWN);

        if (menu_mode == MENU_MODE_EDIT)
        {
            param_adjust(key, -1);  /* 减小值 */
        }
        else
        {
            key = key->next_brother;  /* 下一个兄弟节点 */
        }
        return;
    }
}