#include "zf_common_headfile.h"
#include "zf_driver_dma.h"
#include "my_image_show.h"
#define UART_INDEX              (DEBUG_UART_INDEX   )                           // 默认 UART_1
#define UART_BAUDRATE           (DEBUG_UART_BAUDRATE)                           // 默认 115200
#define UART_TX_PIN             (DEBUG_UART_TX_PIN  )                           // 默认 UART1_TX_A9
#define UART_RX_PIN             (DEBUG_UART_RX_PIN  )                           // 默认 UART1_RX_A10

#define UART_PRIORITY           (UART1_IRQn)                                    // 对应串口中断的中断编号 在 mm32f3277gx.h 头文件中查看 IRQn_Type 枚举体

uint8 uart_get_data[64];                                                        // 串口接收数据缓冲区
uint8 fifo_get_data[64];                                                        // fifo 输出读出缓冲区

uint8 get_data = 0;                                                             // 接收数据变量
uint32 fifo_data_count = 0;                                                     // fifo 数据个数

fifo_struct uart_data_fifo;

int main (void)
{
    clock_init(SYSTEM_CLOCK_120M);                                              // 初始化芯片时钟 工作频率为 120MHz
    debug_init();                                                               // 初始化默认 debug uart

    // 此处编写用户代码 例如外设初始化代码等
    fifo_init(&uart_data_fifo, FIFO_DATA_8BIT, uart_get_data, 64);              // 初始化 fifo 挂载缓冲区
     mt9v03x_init();
     ips200_init(IPS200_TYPE_SPI);            // 初始化 IPS200（SPI 模式）
    ips200_clear();

    uart_init(UART_INDEX, UART_BAUDRATE, UART_TX_PIN, UART_RX_PIN);             // 初始化编码器模块与引脚 正交解码编码器模式
    uart_rx_interrupt(UART_INDEX, ZF_ENABLE);                                   // 开启 UART_INDEX 的接收中断
    interrupt_set_priority(UART_PRIORITY, 0);                                   // 设置对应 UART_INDEX 的中断优先级为 0

    uart_write_string(UART_INDEX, "UART Text.");                                // 输出测试信息
    uart_write_byte(UART_INDEX, '\r');                                          // 输出回车
    uart_write_byte(UART_INDEX, '\n');                                          // 输出换行
    // 此处编写用户代码 例如外设初始化代码等
    static uint8 img_buf[MT9V03X_H][MT9V03X_W];                                  // 本地缓冲，防止 DMA 覆盖

    while(1)
    {
        // 直接显示摄像头原始灰度图像（threshold=0 不做二值化）
        if(mt9v03x_finish_flag)
        {
            // 临界区：关闭场中断 + 关闭 DMA，确保拷贝的是完整一帧
            interrupt_disable(MT9V03X_VSYNC_IRQN);                              // 禁止场中断，防止 VSYNC 重开 DMA
            dma_disable(MT9V03X_DMA_CH);                                        // 确保 DMA 完全停止
            memcpy(img_buf, mt9v03x_image, MT9V03X_IMAGE_SIZE);                 // 拷贝完整一帧到本地缓冲
            mt9v03x_finish_flag = 0;                                             // 清除标志
            interrupt_enable(MT9V03X_VSYNC_IRQN);                               // 恢复场中断（DMA 保持关闭，下一帧 VSYNC 会自动重开）

            // 从本地缓冲显示，慢速刷新确保肉眼可见完整帧
            ips200_show_gray_image(0, 0, (uint8 *)img_buf, MT9V03X_W, MT9V03X_H, MT9V03X_W, MT9V03X_H, 0);
            system_delay_ms(50);                                                 // 降低刷新速度，~20fps
        }

        // 此处编写需要循环执行的代码
        fifo_data_count = fifo_used(&uart_data_fifo);                           // 查看 fifo 是否有数据
        if(0 != fifo_data_count)                                                // 读取到数据了
        {
            fifo_read_buffer(&uart_data_fifo, fifo_get_data, &fifo_data_count, FIFO_READ_AND_CLEAN);    // 将 fifo 中数据读出并清空 fifo 挂载的缓冲
            uart_write_string(UART_INDEX, "\r\nUART get data:");                // 输出测试信息
            uart_write_buffer(UART_INDEX, fifo_get_data, fifo_data_count);      // 将读取到的数据发送出去
        }
        // 此处编写需要循环执行的代码
    }
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     UART_INDEX 的接收中断处理函数 这个函数将在 UART_INDEX 对应的中断调用 详见 isr.c
// 参数说明     void
// 返回参数     void
// 使用示例     uart_rx_interrupt_handler();
//-------------------------------------------------------------------------------------------------------------------
void uart_rx_interrupt_handler (void)
{ 
//    get_data = uart_read_byte(UART_INDEX);                                      // 接收数据 while 等待式 不建议在中断使用
    uart_query_byte(UART_INDEX, &get_data);                                     // 接收数据 查询式 有数据会返回 TRUE 没有数据会返回 FALSE
    fifo_write_buffer(&uart_data_fifo, &get_data, 1);                           // 将数据写入 fifo 中
}
