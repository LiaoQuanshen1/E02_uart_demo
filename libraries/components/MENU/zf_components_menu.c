#include "zf_components_menu.h"
#include "zf_device_ips200.h"
#include "zf_common_font.h"
#include <string.h>
#include <stdio.h>

/* ---- 菜单区域 ---- */
#define MENU_Y_START    240
#define MENU_LINE_H     16          /* IPS200_8X16_FONT 每行高度 */
#define MENU_MAX_LINES  5           /* 80/16 = 5 行 */
#define MENU_CHAR_W     8           /* 8x16 字体字符宽度 */
#define MENU_FULL_W     30          /* 240/8 = 30 字符覆盖全宽 */

MENU_ITEM head, *key;
static menu_state_t menu_state = MENU_CLOSED;

/* ---- 内部辅助 ---- */
static void Create_Menu_Item(MENU_ITEM *father, MENU_ITEM *me, const char name[], void *data, MENU_KIND kind)
{
    if(father->kind != MENU_FOLDER) return;
    me->name       = name;
    me->sons       = 0;
    me->select     = false;
    me->data       = data;
    me->kind       = kind;
    me->father     = father;
    me->first_child = NULL;
    me->prev_brother = NULL;
    me->next_brother = NULL;
    if(father->sons == 0)
    {
        father->first_child = me;
    }
    else
    {
        MENU_ITEM *p = father->first_child;
        while(p->next_brother != NULL && p->next_brother != father->first_child)
            p = p->next_brother;
        p->next_brother = me;
        me->prev_brother = p;
        me->next_brother = father->first_child;
        father->first_child->prev_brother = me;
    }
    me->index = father->sons;
    father->sons++;
}

/* ---- 用 IPS200 清除菜单区域 ---- */
static void menu_clear_area(void)
{
    ips200_set_font(IPS200_8X16_FONT);
    ips200_set_color(RGB565_BLACK, RGB565_BLACK);
    for(uint8_t line = 0; line < MENU_MAX_LINES; line++)
    {
        ips200_show_string(0, MENU_Y_START + line * MENU_LINE_H,
                           "                              "); /* 30 个空格 */
    }
}

/* ---- 格式化参数值到字符串 ---- */
static void format_value(MENU_ITEM *item, char *buf, uint8_t buf_size)
{
    if(item->data == NULL || item->kind == MENU_FOLDER)
    {
        snprintf(buf, buf_size, "...");
        return;
    }
    param_desc_t *p = (param_desc_t *)item->data;
    switch(p->kind)
    {
    case int_Box:    snprintf(buf, buf_size, "%d",   *(int     *)p->value_ptr); break;
    case float_Box:  snprintf(buf, buf_size, "%.1f",  *(float   *)p->value_ptr); break;
    case bool_Box:   snprintf(buf, buf_size, "%s",    *(bool    *)p->value_ptr ? "ON" : "OFF"); break;
    case uint8_Box:  snprintf(buf, buf_size, "%u",   *(uint8_t *)p->value_ptr); break;
    case uint16_Box: snprintf(buf, buf_size, "%u",   *(uint16_t*)p->value_ptr); break;
    case uint32_Box: snprintf(buf, buf_size, "%lu",  (unsigned long)*(uint32_t*)p->value_ptr); break;
    default:         snprintf(buf, buf_size, "?"); break;
    }
}

/* ==================================================================
 * 公开接口
 * ================================================================== */

void Create_Menu_Folder(MENU_ITEM *father, MENU_ITEM *me, const char name[])
{
    Create_Menu_Item(father, me, name, NULL, MENU_FOLDER);
}

void Create_Menu_Number(MENU_ITEM *father, MENU_ITEM *me, const char name[], void *data, MENU_KIND kind, float step)
{
    Create_Menu_Item(father, me, name, data, kind);
    if(data != NULL)
        ((param_desc_t *)data)->step = step;
}

void menu_init(void)
{
    head.data         = NULL;
    head.father       = NULL;
    head.first_child  = NULL;
    head.kind         = MENU_FOLDER;
    head.name         = "MENU";
    head.next_brother = NULL;
    head.prev_brother = NULL;
    head.sons         = 0;
    head.index        = 0;
    key = head.first_child;
    menu_state = MENU_CLOSED;
}

menu_state_t menu_get_state(void)
{
    return menu_state;
}

/* ---- E2: 打开菜单 / 进文件夹 / 开始编辑 / 减小 ---- */
void menu_open(void)
{
    switch(menu_state)
    {
    case MENU_CLOSED:
        if(head.first_child != NULL)
        {
            menu_state = MENU_BROWSING;
            key = head.first_child;
        }
        break;
    case MENU_BROWSING:
        menu_enter();
        break;
    case MENU_EDITING:
        menu_dec();
        break;
    }
}

/* ---- E2 辅助：进入文件夹或开始编辑参数 ---- */
void menu_enter(void)
{
    if(menu_state != MENU_BROWSING) return;
    if(key->kind == MENU_FOLDER && key->sons > 0)
    {
        key = key->first_child;
    }
    else if(key->data != NULL && key->kind != MENU_FOLDER)
    {
        menu_state = MENU_EDITING;
    }
}

/* ---- E3: 返回上级 / 退出菜单 / 取消编辑 / 增大 ---- */
void menu_back(void)
{
    switch(menu_state)
    {
    case MENU_CLOSED:
        break;
    case MENU_BROWSING:
        if(key->father != NULL && key->father != &head)
        {
            key = key->father;
        }
        else
        {
            menu_state = MENU_CLOSED;
            menu_clear_area();
        }
        break;
    case MENU_EDITING:
        menu_inc();
        break;
    }
}

/* ---- E4: 下一个条目 / 确认并切到下一个编辑 ---- */
void menu_next(void)
{
    switch(menu_state)
    {
    case MENU_BROWSING:
        if(key->next_brother != NULL)
            key = key->next_brother;
        break;
    case MENU_EDITING:
        /* 确认当前 → 找下一个可编辑项，找不到则退出编辑 */
        {
            MENU_ITEM *nxt = key->next_brother;
            while(nxt != key)
            {
                if(nxt->data != NULL && nxt->kind != MENU_FOLDER)
                {
                    key = nxt;
                    return;
                }
                nxt = nxt->next_brother;
                if(nxt == NULL) break;
            }
            menu_state = MENU_BROWSING;
        }
        break;
    default:
        break;
    }
}

/* ---- E5: 选中编辑 / 取消编辑 ---- */

/* ---- 参数减小 ---- */
void menu_dec(void)
{
    if(menu_state != MENU_EDITING || key == NULL || key->data == NULL) return;
    param_desc_t *p = (param_desc_t *)key->data;
    float  fv;
    int    iv;

    switch(p->kind)
    {
    case float_Box:
        fv = *(float *)p->value_ptr - p->step;
        if(fv < p->min_val) fv = p->min_val;
        *(float *)p->value_ptr = fv;
        break;
    case int_Box:
        iv = *(int *)p->value_ptr - (int)p->step;
        if(iv < (int)p->min_val) iv = (int)p->min_val;
        *(int *)p->value_ptr = iv;
        break;
    case uint8_Box:
        iv = *(uint8_t *)p->value_ptr - (uint8_t)p->step;
        if(iv < (int)p->min_val) iv = (int)p->min_val;
        *(uint8_t *)p->value_ptr = (uint8_t)iv;
        break;
    case uint16_Box:
        iv = *(uint16_t *)p->value_ptr - (uint16_t)p->step;
        if(iv < (int)p->min_val) iv = (int)p->min_val;
        *(uint16_t *)p->value_ptr = (uint16_t)iv;
        break;
    case uint32_Box:
        {
            int32_t sv = (int32_t)*(uint32_t *)p->value_ptr - (int32_t)p->step;
            if(sv < (int32_t)p->min_val) sv = (int32_t)p->min_val;
            *(uint32_t *)p->value_ptr = (uint32_t)sv;
        }
        break;
    case bool_Box:
        *(bool *)p->value_ptr = false;
        break;
    default:
        break;
    }
}

/* ---- 参数增大 ---- */
void menu_inc(void)
{
    if(menu_state != MENU_EDITING || key == NULL || key->data == NULL) return;
    param_desc_t *p = (param_desc_t *)key->data;
    float  fv;
    int    iv;

    switch(p->kind)
    {
    case float_Box:
        fv = *(float *)p->value_ptr + p->step;
        if(fv > p->max_val) fv = p->max_val;
        *(float *)p->value_ptr = fv;
        break;
    case int_Box:
        iv = *(int *)p->value_ptr + (int)p->step;
        if(iv > (int)p->max_val) iv = (int)p->max_val;
        *(int *)p->value_ptr = iv;
        break;
    case uint8_Box:
        iv = *(uint8_t *)p->value_ptr + (uint8_t)p->step;
        if(iv > (int)p->max_val) iv = (int)p->max_val;
        *(uint8_t *)p->value_ptr = (uint8_t)iv;
        break;
    case uint16_Box:
        iv = *(uint16_t *)p->value_ptr + (uint16_t)p->step;
        if(iv > (int)p->max_val) iv = (int)p->max_val;
        *(uint16_t *)p->value_ptr = (uint16_t)iv;
        break;
    case uint32_Box:
        {
            uint32_t uv = *(uint32_t *)p->value_ptr + (uint32_t)p->step;
            if(uv > (uint32_t)p->max_val) uv = (uint32_t)p->max_val;
            *(uint32_t *)p->value_ptr = uv;
        }
        break;
    case bool_Box:
        *(bool *)p->value_ptr = true;
        break;
    default:
        break;
    }
}

/* ---- E5 action: 选中/取消编辑 ---- */
static void menu_select_action(void)
{
    switch(menu_state)
    {
    case MENU_BROWSING:
        if(key->data != NULL && key->kind != MENU_FOLDER)
            menu_state = MENU_EDITING;
        break;
    case MENU_EDITING:
        menu_state = MENU_BROWSING;
        break;
    default:
        break;
    }
}

/* ---- 供 my_key 调用的 E5 包装 ---- */
void menu_select(void)
{
    menu_select_action();
}

/* ==================================================================
 * 菜单绘制（主循环每帧调用）
 * ================================================================== */
void menu_display(void)
{
    if(menu_state == MENU_CLOSED) return;
    if(key == NULL || key->father == NULL) return;

    MENU_ITEM *parent = key->father;
    uint8_t total = parent->sons;
    if(total == 0) return;

    /* 计算 key 在兄弟中的索引 */
    uint8_t key_idx = 0;
    {
        MENU_ITEM *s = parent->first_child;
        for(uint8_t i = 0; i < total; i++)
        {
            if(s == key) { key_idx = i; break; }
            s = s->next_brother;
        }
    }

    /* 滑动窗口：确保 key 可见 */
    uint8_t win_start = 0;
    uint8_t win_count = total;
    if(total > MENU_MAX_LINES)
    {
        if(key_idx < 2)
            win_start = 0;
        else if(key_idx > total - 3)
            win_start = total - MENU_MAX_LINES;
        else
            win_start = key_idx - 2;
        win_count = MENU_MAX_LINES;
    }

    /* 定位到窗口第一项 */
    MENU_ITEM *s = parent->first_child;
    for(uint8_t i = 0; i < win_start; i++)
        s = s->next_brother;

    /* 设置字体和颜色 */
    ips200_set_font(IPS200_8X16_FONT);

    /* 逐行绘制 */
    for(uint8_t line = 0; line < win_count; line++)
    {
        uint16 y  = MENU_Y_START + line * MENU_LINE_H;
        char buf[31]; /* 30 chars + null */
        char val[13];
        memset(buf, ' ', sizeof(buf));

        /* 光标标记 */
        char cursor = ' ';
        if(s == key)
            cursor = (menu_state == MENU_EDITING) ? '*' : '>';

        /* 参数值 */
        format_value(s, val, sizeof(val));

        /* 编辑态高亮色，浏览态普通色 */
        if(s == key && menu_state == MENU_EDITING)
            ips200_set_color(RGB565_YELLOW, RGB565_BLACK);
        else if(s == key)
            ips200_set_color(RGB565_WHITE, RGB565_BLUE);
        else
            ips200_set_color(RGB565_WHITE, RGB565_BLACK);

        /* 格式化：X name____________: value */
        snprintf(buf, sizeof(buf), "%c%-14s: %s", cursor, s->name, val);

        ips200_show_string(0, y, buf);

        s = s->next_brother;
    }

    /* 未使用的行用黑色填充，防止残留 */
    ips200_set_color(RGB565_BLACK, RGB565_BLACK);
    for(uint8_t line = win_count; line < MENU_MAX_LINES; line++)
    {
        ips200_show_string(0, MENU_Y_START + line * MENU_LINE_H,
                           "                              ");
    }
}