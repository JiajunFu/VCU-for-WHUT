#include"MC9S12XS128.h"
#include "derivative.h"      /* derivative-specific definitions */
#include "IO.h"


 // A��Ϊ���������

 // B��Ϊ����������

/*************************************************************/
/*                       IO�ڳ�ʼ��                          */
/*************************************************************/
 void IO_Init(void)
{ //unsigned char  on_fire_low, on_fire_high,  start;


   W_DR=0xff;                       // A�������������W
  // RDRIV_RDPA=0x1;                  // A�ڽ���������
   W=0x00;                          // A�ڳ�ʼ��Ϊ0x00����Ӧ�Ĺ������Ϊ0
   
   
   SIN_DR=0x00;                     // B�����룬������SIN
   PUCR_PUPBE=0x1;                  // B��ʹ����������
   SIN=0x00;                        // B�ڳ�ʼ��Ϊ0xff����Ӧ�ⲿ�����������ź�
   
  
   //on_fire_low = SIN_1=0;
   //on_fire_high = SIN_2=0;
   //start = SIN_3=1;              //�����ⲿ�����ź�
  
  
  
}