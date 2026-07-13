#include "zf_components_menu.h"
#include "zf_device_oled.h"


MENU_ITEM head, *key;
//节点创建
static void Create_Menu_Item(MENU_ITEM *father,MENU_ITEM *me,const char name[],void *data,MENU_KIND kind)
{
    if(father->kind!=MENU_FOLDER)
    return;//只有文件夹下面才可以创建成员
    me->name = name;
    me->sons=0;
    me->select=false;
    me->data=data;
    me->kind=kind;
    me->father = father;
    me->first_child = NULL;
    me->prev_brother = NULL;
    me->next_brother = NULL;
    if(father->sons==0)
    {
    father->first_child=me;
    }
    else
    {
    MENU_ITEM *p=father->first_child;
    while(p->next_brother!=NULL && p->next_brother!=father->first_child)//因为根本找不到最后节点
        p=p->next_brother;//找到同一层级的最后节点
    p->next_brother=me;
    me->prev_brother=p;//建立同一层级的连接
    me->next_brother=father->first_child;
    father->first_child->prev_brother=me;//建立环形链表，防止指针跑飞
    }
    me->index=father->sons;
    father->sons++;
}
void Create_Menu_Folder(MENU_ITEM *father,MENU_ITEM *me,const char name[])
{
    Create_Menu_Item(father,me,name,NULL,MENU_FOLDER);
}
void Create_Menu_Number(MENU_ITEM *father,MENU_ITEM *me,const char name[],void *data,MENU_KIND kind)
{
    Create_Menu_Item(father,me,name,data,kind);
}



void menu_init(void) {
    /* 初始化根节点 */
    head.data        = NULL;
    head.father      = NULL;
    head.first_child = NULL;
    head.kind        = MENU_FOLDER;
    head.name        = "MENU";
    head.next_brother= NULL;
    head.prev_brother= NULL;
    head.sons        = 0;
    head.index       = 0;

//    /* 二级文件夹 */
//    Create_Menu_Folder(&head, &m_bal, "Balance");
//    Create_Menu_Folder(&head, &m_spd, "Speed  ");
//    Create_Menu_Folder(&head, &m_tgt, "Target ");

//    /* Balance PID 子项 */
//    Create_Menu_Number(&m_bal, &m_bal_kp, "Kp", &p_bal_kp, float_Box);
//    Create_Menu_Number(&m_bal, &m_bal_ki, "Ki", &p_bal_ki, float_Box);
//    Create_Menu_Number(&m_bal, &m_bal_kd, "Kd", &p_bal_kd, float_Box);

//    /* Speed PI 子项 */
//    Create_Menu_Number(&m_spd, &m_spd_kp, "Kp", &p_spd_kp, int_Box);
//    Create_Menu_Number(&m_spd, &m_spd_ki, "Ki", &p_spd_ki, int_Box);

//    /* Target 子项 */
//    Create_Menu_Number(&m_tgt, &m_tgt_offs, "Offs", &p_tgt_off, float_Box);
//使用示例

    key = head.first_child;  /* 默认选中 "Balance" */
}
void number_show(void) {
  MENU_ITEM *h = key->father;
  MENU_ITEM *s = h->first_child;
  for (int i = 1; i <= h->sons; i++) {
    if (s->data == NULL) {
        s = s->next_brother;
        continue;  /* 文件夹无数据 */
    }
    param_desc_t *p = (param_desc_t *)s->data;
    switch (p->kind) {
    case int_Box:
      //oled_show_printf(50, i * 8, OLED_6X8, "%6d", *(int *)(p->value_ptr));
      break;
    case float_Box:
      //oled_show_printf(50, i * 8, OLED_6X8, "%7.1f", *(float *)(p->value_ptr));
      break;
    case bool_Box:
      //oled_show_printf(50, i * 8, OLED_6X8, *(bool *)(p->value_ptr) ? "Y" : "N");
      break;
    case uint8_Box:
      //oled_show_printf(50, i * 8, OLED_6X8, "%3u", *(uint8_t *)(p->value_ptr));
      break;
    case uint16_Box:
      //oled_show_printf(50, i * 8, OLED_6X8, "%5u", *(uint16_t *)(p->value_ptr));
      break;
    case uint32_Box:
      //oled_show_printf(50, i * 8, OLED_6X8, "%6lu", *(uint32_t *)(p->value_ptr));
      break;
    default:
      break;
    }
    s = s->next_brother;
  }
}

void key_show(void) {
  MENU_ITEM *h = key->father;
  MENU_ITEM *s = h->first_child;
  for (int i = 1; i <= h->sons; i++) {//根据sons大小遍历，不会无线循环卡死
    if (s == key) {
      //oled_show_printf(0, i * 8, OLED_6X8, "->");
    } 
    else {
      //oled_show_printf(0, i * 8, OLED_6X8, "  ");//擦掉箭头
    }
    s = s->next_brother;
  }
}
void key_down(void)
{
    key=key->next_brother;
}
void key_up(void)
{
    key=key->prev_brother;//清除箭头已经在别处实现
}
void key_enter(void)
{
    if(key->sons>0)
    {
        key=key->first_child;
        oled_clear();
    }
}
void key_quit(void)
{
    if(key->father->father!=NULL)
    {key=key->father;
    oled_clear();//菜单变化时记得清除屏幕
    }
}
void key_select(void)
{
    if(key->kind!=MENU_FOLDER&&key->kind!=bool_Box)
    key->select=!key->select;
}//可以根据是否选中，来选择是否调这个参数
void key_plus(void)
{
    param_desc_t *p;
    float  fv;
    int    iv;

    /* 仅当当前项绑定了参数描述符时才可调节 */
    if (key == NULL || key->data == NULL)
        return;

    p = (param_desc_t *)key->data;

    switch (p->kind)
    {
    case float_Box:
        fv = *(float *)p->value_ptr + p->step;
        if (fv > p->max_val) fv = p->max_val;
        *(float *)p->value_ptr = fv;
        break;

    case int_Box:
        iv = *(int *)p->value_ptr + (int)p->step;
        if (iv > (int)p->max_val) iv = (int)p->max_val;
        *(int *)p->value_ptr = iv;
        break;

    case uint8_Box:
        iv = *(uint8_t *)p->value_ptr + (uint8_t)p->step;
        if (iv > (int)p->max_val) iv = (int)p->max_val;
        *(uint8_t *)p->value_ptr = (uint8_t)iv;
        break;

    case uint16_Box:
        iv = *(uint16_t *)p->value_ptr + (uint16_t)p->step;
        if (iv > (int)p->max_val) iv = (int)p->max_val;
        *(uint16_t *)p->value_ptr = (uint16_t)iv;
        break;

    case uint32_Box:
    {
        uint32_t uv = *(uint32_t *)p->value_ptr + (uint32_t)p->step;
        if (uv > (uint32_t)p->max_val) uv = (uint32_t)p->max_val;
        *(uint32_t *)p->value_ptr = uv;
        break;
    }

    case bool_Box:
        *(bool *)p->value_ptr = true;
        break;

    default:
        break;
    }
}//调参也许还有问题
void menu_show(void) {
  MENU_ITEM *h = key->father;
  MENU_ITEM *s = h->first_child;
  for (int i = 1; i <= h->sons; i++) {
//    oled_show_printf(13, i * 8, OLED_6X8, "%s",
//                s->name); // 需要跟具体的驱动对接，其他模块需要自己动手
    s = s->next_brother;
  }
  s = h->first_child;
  s = s->first_child;
  h = h->first_child;
  for (int i = 1; i <= h->sons; i++) {
//    oled_show_printf(13, i * 8, OLED_6X8, "%s",
//                s->name); // 需要跟具体的驱动对接，其他模块需要自己动手
    s = s->next_brother;
  }
  number_show();
  key_show();
}