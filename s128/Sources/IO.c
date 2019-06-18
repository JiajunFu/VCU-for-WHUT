#include"MC9S12XS128.h"
#include "derivative.h"      /* derivative-specific definitions */
#include "IO.h"


 // A口为开关量输出

 // B口为开关量输入

/*************************************************************/
/*                       IO口初始化                          */
/*************************************************************/
 void IO_Init(void)
{ //unsigned char  on_fire_low, on_fire_high,  start;


   W_DR=0xff;                       // A口输出，开关量W
  // RDRIV_RDPA=0x1;                  // A口降功率驱动
   W=0x00;                          // A口初始化为0x00，对应的光耦输出为0
   
   
   SIN_DR=0x00;                     // B口输入，开关量SIN
   PUCR_PUPBE=0x1;                  // B口使能上拉功能
   SIN=0x00;                        // B口初始化为0xff，对应外部光耦无输入信号
   
  
   //on_fire_low = SIN_1=0;
   //on_fire_high = SIN_2=0;
   //start = SIN_3=1;              //都是外部输入信号
  
  
  
}
void gpio_set (char IO,uint8 data)
{
    if(data == 0)
    {
        IO=0x00;// GPIO PDOR 管脚号 清0，即对应管脚配置为端口输出低电平
    }
    else
    {
        IO=0x01; // GPIO PDOR 管脚号 置1，即对应管脚配置为端口输出高电平
    }
}

uint8 gpio_get(IO)
{
    return IO;// ?? GPIO PDIR ptxn ??,?????????
}