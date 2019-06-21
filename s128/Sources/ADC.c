#include <MC9S12XS128.h>
#include "ADC.h"

unsigned char AD_value[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char c=0;
/*************************************************************/
/*                       ADC��ʼ��                           */
/*************************************************************/
void ADC_Init(void) 
{
 
  ATD0CTL3 = 0x08;            // ��ͨ����ת�����ӳ��
  ATD0CTL4 = 0xb7;            // ѡ��8λ����ģ��ת��,ADģ��ʱ��Ƶ��Ϊ2MHz
            
  ATD0CTL2 = 0x40;            // ����A/Dת��,��������,��ֹ�ж�
}                                          

/*************************************************************/
/*                        ��ADת��                         */
/*************************************************************/
uint16 ADCValue(uint8 channel)
    {
      uint16 temp;                        //�ݴ�A/Dת���Ľ��
    	ATD0CTL5 = channel;

    	                                    //ȡA/Dת�����
    //	while (1)
      for(;;)                                    
    	if ((ATD0STAT0&(1<< 7)) != 0)       //�ж�ATDSTAT0�ĵ�7λ�Ƿ�Ϊ1
    	{
    	    temp = ATD0DR0;                 //��A/D���ݼĴ���0�ж�12λ����
    	    break;
    	}
    	return  temp;
    }


void AD_Get_value(void)
{
               
             
 while(!ATD0STAT0_SCF);               // �ȴ�ADת��
                                 
  {if(c==0) 
    {
     ATD0CTL5 = 0x01;    //ת��AD01
   
    AD_value[1] = ATD0DR0;
       
    }                                  // ACC_1
   else if(c==1) 
    {
     ATD0CTL5 = 0x02;    //ת��AD02
   
    AD_value[2]= ATD0DR0;
      
               // ACC_2
    }
       
  }
  c++;  
  if(c>1)
   c=0;
}

 