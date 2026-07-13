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


void menu_init(void);
void menu_show(void);
void number_show(void);
void key_show(void);
void key_down(void);
void key_up(void);
void key_quit(void);
void key_enter(void);
void key_select(void);
void key_plus(void);

static void Create_Menu_Item(MENU_ITEM *,MENU_ITEM *,const char [],void *,MENU_KIND);
void Create_Menu_Folder(MENU_ITEM *,MENU_ITEM *,const char []);
void Create_Menu_Number(MENU_ITEM *,MENU_ITEM *,const char [],void *,MENU_KIND);

#endif