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
            
  ATD0CTL2 = 0x40;            // 启动A/D转换,快速清零,禁止中断
}                                          

/*************************************************************/
/*                        起动AD转换                         */
/*************************************************************/
uint16 ADCValue(uint8 channel)
    {
      uint16 temp;                        //暂存A/D转换的结果
    	ATD0CTL5 = channel;

    	                                    //取A/D转换结果
    //	while (1)
      for(;;)                                    
    	if ((ATD0STAT0&(1<< 7)) != 0)       //判断ATDSTAT0的第7位是否为1
    	{
    	    temp = ATD0DR0;                 //从A/D数据寄存器0中读12位数据
    	    break;
    	}
    	return  temp;
    }


void AD_Get_value(void)
{
               
             
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

 