
#include "zf_device_virtual_oscilloscope.h"

uint8 virtual_oscilloscope_data[10];

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     CRC 校验
// 参数说明     buff            需要进行 CRC 计算的数据地址
// 参数说明     crc_cnt         需要进行 CRC 计算的数据个数
// 返回参数     uint16          CRC 校验结果
// 使用示例     crc_16  = crc_check(virtual_oscilloscope_data, 8);
// 备注信息     内部使用 用户无需关心
//-------------------------------------------------------------------------------------------------------------------
static uint16 crc_check (uint8 *buff, uint8 crc_cnt)
{
    uint16 crc_temp;
    uint8 i, j;
    crc_temp = 0xffff;

    for(i = 0; i < crc_cnt; i ++)
    {
        crc_temp ^= buff[i];
        for(j = 0; 8 > j; j ++)
        {
            if(crc_temp & 0x01)
            {
                crc_temp = (crc_temp >> 1) ^ 0xa001;
            }
            else
            {
                crc_temp = crc_temp >> 1;
            }
        }
    }
    return(crc_temp);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     虚拟示波器数据转换函数
// 参数说明     data1           要发送的第一个数据
// 参数说明     data2           要发送的第二个数据
// 参数说明     data3           要发送的第三个数据
// 参数说明     data4           要发送的第四个数据
// 返回参数     void
// 使用示例     uint8 data_buffer[10];
//              virtual_oscilloscope_data_conversion(100, 200, 300, 400, data_buffer);
//              wireless_uart_send_buff(data_buffer, 10);
// 备注信息     这个函数不带发送 他只是处理数据
//-------------------------------------------------------------------------------------------------------------------
void virtual_oscilloscope_data_conversion (const int16 data1, const int16 data2, const int16 data3, const int16 data4)
{
    uint16 crc_16 = 0;

    virtual_oscilloscope_data[0] = (uint8)((uint16)data1 & 0xff);
    virtual_oscilloscope_data[1] = (uint8)((uint16)data1 >> 8);

    virtual_oscilloscope_data[2] = (uint8)((uint16)data2 & 0xff);
    virtual_oscilloscope_data[3] = (uint8)((uint16)data2 >> 8);

    virtual_oscilloscope_data[4] = (uint8)((uint16)data3 & 0xff);
    virtual_oscilloscope_data[5] = (uint8)((uint16)data3>>8);

    virtual_oscilloscope_data[6] = (uint8)((uint16)data4 & 0xff);
    virtual_oscilloscope_data[7] = (uint8)((uint16)data4 >> 8);

    crc_16  = crc_check(virtual_oscilloscope_data, 8);
    virtual_oscilloscope_data[8] = (uint8)(crc_16 & 0xff);
    virtual_oscilloscope_data[9] = (uint8)(crc_16 >> 8);
}
