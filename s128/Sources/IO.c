#include"MC9S12XS128.h"
#include "derivative.h"      /* derivative-specific definitions */
#include "IO.h"
#include "lib.h"


 // A��Ϊ���������

 // B��Ϊ����������

/*************************************************************/
/*                       IO�ڳ�ʼ��                          */
/*************************************************************/
 void IO_Init(void)
{ //unsigned char  on_fire_low, on_fire_high,  start;

                 //��ʼ���������͵�
  GPIO_Init (W1,0,1,0);
  GPIO_Init (W3,2,1,0);  
  GPIO_Init (W2,1,1,0);  

                //��ʼ�������ź�
  GPIO_Init(S1IN,2,0,0);
  GPIO_Init(S2IN,1,0,0);
  GPIO_Init(S3IN,0,0,0);

               //��ʼ������ź�
  GPIO_Init(W4,3,1,0);
  GPIO_Init(W5,4,1,0);
  GPIO_Init(W6,5,1,0);
  GPIO_Init(W7,6,1,0);
 

 

}