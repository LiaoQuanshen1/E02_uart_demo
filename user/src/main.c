#include "my_image_show.h"
#include "my_motor.h"
#include "my_key.h"
#include "zf_common_headfile.h"
#include "zf_components_menu.h"

#define UART_INDEX (DEBUG_UART_INDEX)       // 默认 UART_1
#define UART_BAUDRATE (DEBUG_UART_BAUDRATE) // 默认 115200
#define UART_TX_PIN (DEBUG_UART_TX_PIN)     // 默认 UART1_TX_A9
#define UART_RX_PIN (DEBUG_UART_RX_PIN)     // 默认 UART1_RX_A10

#define UART_PRIORITY                                                          \
  (UART1_IRQn) // 对应串口中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type
               // 枚举体

uint8 uart_get_data[64]; // 串口接收数据缓冲区
uint8 fifo_get_data[64]; // fifo 输出读出缓冲区

uint8 get_data = 0;         // 接收数据变量
uint32 fifo_data_count = 0; // fifo 数据个数

fifo_struct uart_data_fifo;

// /* ---- 菜单演示数据 ---- */
// static MENU_ITEM m_pid, m_pid_kp, m_pid_ki, m_pid_kd;
// static MENU_ITEM m_speed;

// static float    g_pid_kp  = 0.0f;
// static float    g_pid_ki  = 0.0f;
// static float    g_pid_kd  = 0.0f;
// static int      g_speed   = 50;

// static param_desc_t p_pid_kp  = { &g_pid_kp, float_Box, 0, 0.0f, 100.0f };
// static param_desc_t p_pid_ki  = { &g_pid_ki, float_Box, 0, 0.0f, 10.0f  };
// static param_desc_t p_pid_kd  = { &g_pid_kd, float_Box, 0, 0.0f, 50.0f  };
// static param_desc_t p_speed   = { &g_speed,  int_Box,   0, 0,     100    };
static int g_speed = 0,g_count = 0;
static float g_ajust = 0.0f;
static MENU_ITEM m_speed, m_ajust, m_count;

static param_desc_t p_speed = { &g_speed, int_Box, 5, -100, 100 };
static param_desc_t p_ajust = { &g_ajust, float_Box, 0.1, -10, 10 };
static param_desc_t p_count = { &g_count, int_Box, 5, -100, 100 };

static void menu_setup(void)
{
    menu_init();
    Create_Menu_Number(&head, &m_speed, "Speed", &p_speed);
    Create_Menu_Number(&head, &m_ajust, "Ajust", &p_ajust);
    Create_Menu_Number(&head, &m_count, "Count", &p_count);
}

int main(void) {
  clock_init(SYSTEM_CLOCK_120M); // 初始化芯片时钟 工作频率为 120MHz
  debug_init();                  // 初始化默认 debug uart

  // 此处编写用户代码 例如外设初始化代码等
  motor_init(); // 初始化电机驱动模块
 
  fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data,
            64); // 初始化 fifo 挂载缓冲区
  mt9v03x_init();
  ips200_init(IPS200_TYPE_SPI); // 初始化 IPS200（SPI 模式）
  ips200_clear();

  uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN,
            UART_RX_PIN); // 初始化编码器模块与引脚 正交解码编码器模式
  uart_rx_interrupt(UART_INDEX, ZF_ENABLE); // 开启 UART_INDEX 的接收中断
  interrupt_set_priority(UART_PRIORITY,
                         0); // 设置对应 UART_INDEX 的中断优先级为 0

  uart_write_string(UART_INDEX, "UART Text."); // 输出测试信息
  uart_write_byte(UART_INDEX, '\r');           // 输出回车
  uart_write_byte(UART_INDEX, '\n');           // 输出换行
  // 此处编写用户代码 例如外设初始化代码等
  // motor_b_set(10);
  // motor_a_set(10);

  my_key_init();  // 初始化按键（E2/E3/E4/E5）
  menu_setup();   // 初始化菜单系统 + 创建演示参数

  while (1) {
    // 获取编码器计数值（仅用于清除编码器中断标志位）
    image_show(); // 图像采集 + 原始灰度 + 二值化 + 巡线，全部封装在模块内
    my_key_process(); // 按键扫描 + 菜单操作
motor_a_set(g_speed); // 电机 A 速度设置
motor_b_set(g_speed+g_ajust); // 电机 B 速度设置
    menu_display();   // 菜单绘制（仅在菜单打开时绘制底部区域）

    // 此处编写需要循环执行的代码
    fifo_data_count = fifo_used(&uart_data_fifo); // 查看 fifo 是否有数据
    if (0 != fifo_data_count)                     // 读取到数据了
    {
      fifo_read_buffer(
          &uart_data_fifo, fifo_get_data, &fifo_data_count,
          FIFO_READ_AND_CLEAN); // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
      uart_write_string(UART_INDEX, "\r\nUART get data:"); // 输出测试信息
      uart_write_buffer(UART_INDEX, fifo_get_data,
                        fifo_data_count); // 将读取到的数据发送出去
    }
    // 此处编写需要循环执行的代码
  }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART_INDEX 的接收中断处理函数 这个函数将在 UART_INDEX
// 对应的中断调用 详见 isr.c 参数说明     void 返回参数     void 使用示例
// uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt_handler(void) {
  //    get_data = uart_read_byte(UART_INDEX); // 接收数据 while 等待式
  //    不建议在中断使用
  uart_query_byte(
      UART_INDEX,
      &get_data); // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
  fifo_write_buffer(&uart_data_fifo, &get_data, 1); // 将数据写入 fifo 中
}
