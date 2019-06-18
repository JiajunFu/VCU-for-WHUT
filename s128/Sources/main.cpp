#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "lib.h"
typedef unsigned char uint8;
/*********����̤������********/
unsigned int ACC_1;      //���Ŵ�����1
unsigned int ACC_2;      //���Ŵ�����2
unsigned int Torque;     //ת��
void Init_all (void); 
void drive (void);        



/*************************************************************/
/*                     ��ʼ������                            */
/*************************************************************/
void Init_all(void) 
{
  IO_Init();
  INIT_PLL();
  ECT_Init();
  ADC_Init();
  CAN_Init();
  PIT_Init();
}
/*************************************************************/
/*                         ��ʱ����                          */
/*************************************************************/
void delay1ms(unsigned int n) 
{
    unsigned int i;
    for(i=0;i<n;i++) 
    {
        TFLG1_C0F = 1;              // �����־λ
        TC0 = TCNT + 250;           // ��������Ƚ�ʱ��Ϊ1ms   4us*250 //��Ҫ��n����Ϊ3000����Ӧ����3��
        while(TFLG1_C0F == 0);      // �ȴ���ֱ����������Ƚ��¼�
    }
}

/*************************************************************/
/*                         ������                            */
/*************************************************************/
void main(void) {
uint8 Start_flag=0;                                                           //��ʼ��־λ����
  uint8 Pre_flag=0;                                                             //׼����־λ����
  Init_all();                                                               //�����ж����ȼ�
  while (Pre_flag==0)                                                        //Ԥ��
      {
        if (GPIO_Get(S1IN,PRT,2)==1)
         {
          GPIO_Set(W4,PRT,3,1);
          Pre_flag=1;
         }    
      }
  while (Start_flag==0)                                                      //�жϵ�һ�ν����ʻ״̬
      {
        if (GPIO_Get(S2IN,PRT,1)==1)
         {
          GPIO_Set(W5,PRT,4,1);
          GPIO_Set(W1,PRT,0,1);
          delay1ms(3000);
          GPIO_Set(W1,PRT,0,0);
          Start_flag=1;
         }
      }
  while(1) {
          	if(Start_flag)
			{
				Drive();
			}
			else
			{
				Torque=0;
			}
  
  }
  
  /* put your own code here */
  //EnableInterrupts;
  //for(;;) {
    //_FEED_COP(); /* feeds the dog */
  //} 
  
}

/*************************************************************/
/*                         ��ʻ����                          */
/*************************************************************/
void drive (void)
{
  uint16 PedUpLimit=4095;                                               //����̤���ϼ���λ�ã����Լ����ã�
  uint16 PedDownLimit=0;                                                //����̤���¼���λ�ã����Լ����ã�
  uint16 PedMax=200; 
   ACC_1= ((unsigned int)(AD_value[1]);
   ACC_2= ((unsigned int)(AD_value[2]);                                                      //����̤��������ֵ�����ݾ���������ã�
  if (ACC_1<PedDownLimit||ACC_2<PedDownLimit||ACC_1>PedUpLimit||ACC_2>PedUpLimit)    //�жϼ���̤���Ƿ񳬳�
     Torque=0;
  else if (fabs(ACC_1-ACC_2)>PedMax)                             //�ж�����Ƿ����
     Torque=0;
  else if (Brake_flag==1)
     Torque=0;
  else
     {
       if (GPIO_Get(S3IN,PRT,0)==1)                                             //�ж��Ƿ��µ�����ť
          {
            GPIO_Set(W6,PRT,5,0);
            GPIO_Set(W7,PRT,6,1);
            GPIO_Set(W3,PRT,2,1);                                            //��������
            GPIO_Set(W2,PRT,1,1);;                                            //��������
            Torque=((ACC_1+ACC_2)/2-PedDownLimit)*4095/(PedUpLimit-PedDownLimit)
          }
       else
          {
            GPIO_Set(W6,PRT,5,1);
            GPIO_Set(W7,PRT,6,0);
            GPIO_Set(W3,PRT,2,0);                                            //��������
            GPIO_Set(W2,PRT,1,0);;                                            //��������
            Torque=((ACC_1+ACC_2)/2-PedDownLimit)*4095/(PedUpLimit-PedDownLimit)
          }
     }
     
      /* PWM���torqueֵ�����
}

