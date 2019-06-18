#include <MC9S12XS128.h>
#include "ADC.h"

unsigned char AD_value[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char c=0;
/*************************************************************/
/*                       ADC初始化                           */
/*************************************************************/
void ADC_Init(void) 
{
 
  ATD0CTL3 = 0x08;            // 单通道，转换结果映射
  ATD0CTL4 = 0xb7;            // 选用8位精度模数转换,AD模块时钟频率为2MHz
  ATD0CTL5 = 0xa0;            // AD0、AD1、AD2三个通道采样,右对齐模式
  ATD0CTL2 = 0x40;            // 启动A/D转换,快速清零,禁止中断
}                                          *

/*************************************************************/
/*                        起动AD转换                         */
/*************************************************************/

void AD_Get_value(void)
{
  ATD0CTL5 =ATD0CTL5+c;               // 启动转换
             
 while(!ATD0STAT0_SCF);               // 等待AD转换
                                 
  {if(c==0) 
    {
     ATD0CTL5 = 0x01;    //转换AD01
   
    AD_value[1] = ATD0DR0;
       
    }                                  // ACC_1
   else if(c==1) 
    {
     ATD0CTL5 = 0x02;    //转换AD02
   
    AD_value[2]= ATD0DR0;
      
               // ACC_2
    }
       
  }
  c++;  
  if(c>1)
   c=0;
}

 