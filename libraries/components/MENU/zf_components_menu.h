#ifndef __COMMEN_MEMORY_H
#define __COMMEN_MEMORY_H
#include "zf_common_typedef.h"

/* 参数描述符：描述一个可通过菜单调节的参数 */
typedef enum MENU_KIND{
    MENU_FOLDER=0,
    int_Box,
    float_Box,
    bool_Box,
    uint8_Box,
    uint16_Box,
    uint32_Box,
}MENU_KIND;
typedef struct {
    void       *value_ptr;   /* 指向要调节的变量 */
    MENU_KIND   kind;        /* 数据类型 */
    float       step;        /* 步长 */
    float       min_val;     /* 最小值 */
    float       max_val;     /* 最大值 */
} param_desc_t;

typedef struct MENU_ITEM
{
    const char *name;
    void *data;//指向存放的遍历
    MENU_KIND kind;//变量的类型 
    uint8_t sons;//子节点数目
    uint8_t index;//是父节点的第几个子节点
    bool select;//是否选中
    struct MENU_ITEM *father;
    struct MENU_ITEM *first_child;
    struct MENU_ITEM *prev_brother;
    struct MENU_ITEM *next_brother;
} MENU_ITEM;

/* 菜单运行状态 */
typedef enum {
    MENU_CLOSED,    /* 菜单关闭，正常显示图像 */
    MENU_BROWSING,  /* 浏览模式：上下切换条目 */
    MENU_EDITING,   /* 编辑模式：增减参数值     */
} menu_state_t;

/* 根节点（供外部创建菜单项时引用） */
extern MENU_ITEM head;

/* ---- 节点创建 ---- */
void menu_init(void);
void Create_Menu_Folder(MENU_ITEM *father, MENU_ITEM *me, const char name[]);
void Create_Menu_Number(MENU_ITEM *father, MENU_ITEM *me, const char name[], void *data, MENU_KIND kind, float step);

/* ---- 菜单操作（供 my_key 调用）---- */
menu_state_t menu_get_state(void);
void menu_open(void);       /* 打开菜单              */
void menu_back(void);       /* 返回/退出/取消编辑     */
void menu_next(void);       /* 下一个条目 / 确认切下个 */
void menu_enter(void);      /* 进文件夹 / 开始编辑    */
void menu_select(void);     /* 选中/取消编辑          */
void menu_dec(void);        /* 参数值减小            */
void menu_inc(void);        /* 参数值增大            */

/* ---- 菜单绘制（主循环调用）---- */
void menu_display(void);

#endif