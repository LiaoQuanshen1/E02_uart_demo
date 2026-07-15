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
    void *data;              /* 指向 param_desc_t（数值项）或 NULL（文件夹） */
    MENU_KIND kind;          /* 变量的类型 */
    uint8_t sons;            /* 子节点数目 */
    uint8_t index;           /* 是父节点的第几个子节点 */
    bool select;             /* 是否处于编辑选中状态 */
    struct MENU_ITEM *father;
    struct MENU_ITEM *first_child;
    struct MENU_ITEM *prev_brother;
    struct MENU_ITEM *next_brother;
} MENU_ITEM;

/* 菜单交互模式 */
typedef enum {
    MENU_MODE_NAVIGATE = 0,  /* 浏览模式：上下切换条目、进入/退出文件夹 */
    MENU_MODE_EDIT,           /* 编辑模式：增减参数值               */
} menu_mode_t;

/* ==================== 菜单显示配置 ==================== */
#define MENU_DISPLAY_Y          204     /* 菜单区域起始 Y 坐标（图像下方）    */
#define MENU_LINE_HEIGHT        8       /* 每行高度（IPS200_6X8_FONT 字体）   */
#define MENU_MAX_VISIBLE        4       /* 最多可见行数（标题1行+参数3行）     */
#define MENU_TITLE_COLOR        (RGB565_YELLOW)  /* 标题栏颜色 */
#define MENU_SELECT_COLOR       (RGB565_GREEN)   /* 当前选中项文字颜色  */
#define MENU_NORMAL_COLOR       (RGB565_WHITE)   /* 普通项文字颜色      */
#define MENU_EDIT_COLOR         (RGB565_RED)     /* 编辑模式值颜色      */
#define MENU_BG_COLOR           (RGB565_BLACK)   /* 菜单背景色          */
#define MENU_MAX_PARAM_POOL     20              /* 最大参数描述符池大小 */

/* ==================== 按键映射（KEY_LIST = { E2, E3, E4, E5 }） ==================== */
#define MENU_KEY_UP             KEY_1           /* E2: 上 / 增大值     */
#define MENU_KEY_DOWN           KEY_2           /* E3: 下 / 减小值     */
#define MENU_KEY_ENTER          KEY_3           /* E4: 进入 / 确认     */
#define MENU_KEY_BACK           KEY_4           /* E5: 返回 / 退出     */

void menu_init(void);
void menu_show(void);
void menu_key_handler(void);               /* 在主循环中调用，处理按键输入 */
menu_mode_t menu_get_mode(void);           /* 获取当前菜单模式             */

/* 创建菜单项 */
void Create_Menu_Folder(MENU_ITEM *father, MENU_ITEM *me, const char name[]);

/* 创建数值型菜单项：step=调节步长, min_val=最小值, max_val=最大值 */
void Create_Menu_Number(MENU_ITEM *father, MENU_ITEM *me, const char name[],
                        void *value_ptr, MENU_KIND kind,
                        float step, float min_val, float max_val);

#endif