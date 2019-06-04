#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include"PIT.h"
#include"CAN.h"
#include"ADC.h"
unsigned char     a=0 ,b=0;

/*************************************************************/
/*                     PIT��ʼ��                             */
/*************************************************************/
/* void PIT_Init()
 {
     PITCFLMT=0;       //��ʹ�������ж϶�ʱ��
     PITCE_PCE0=1;     //��0ͨ������������ 
     PITMUX_PMUX0=0;   //��0ͨ��ʹ��΢������0
     PITMTLD0=249;     //Ϊ0ͨ��8λ��������ֵ
     PITLD0=63;     //Ϊ0ͨ��16λ��������ֵ   //(249+1)*(127+1)=32000����������=2ms,ÿ��2ms�ж�һ��
     
     PITINTE_PINTE0=1; //0ͨ����ʱ����ʱ�жϱ�ʹ��
     PITCFLMT=0X80;    //ʹ�������ж϶�ʱ��
     
 }  */

/*************************************************************/
/*                     PIT��ʼ��                             */
/*************************************************************/
 void PIT_Init()
 {
 
   PITMTLD0=249;     //Ϊ0ͨ��8λ��������ֵ
   PITLD0=255;       //Ϊ0ͨ��16λ��������ֵ   
                     //(249+1)*(1279+1)=320000����������,10ms,
                     //����1��2֮��ʵ��ʱ��Ϊ20/3ms������1��2ʱ��Ϊ20ms  ,255��Ӧ2ms
   PITMUX_PMUX0=0;   //��0ͨ��ʹ��΢������0
   PITCE_PCE0=1;     //��0ͨ������������ 
   PITCFLMT=0X80;    //ʹ�������ж϶�ʱ��
   PITINTE_PINTE0=1; //0ͨ����ʱ����ʱ�жϱ�ʹ��
 }
/*************************************************************/
/*                     PIT��ʱ�ж�                           */
/*************************************************************/ 
#pragma CODE_SEG __NEAR_SEG NON_BANKED
interrupt VectorNumber_Vpit0 void PIT_66U(void) 

{  

          b++;
  if(((PITTF_PTF0==1)&&(b==1)))      //2ms�ɼ�һ��ģ���� ��ACC_1����ACC_2
  {   
       b=0;
       a++;
       AD_Get_value();
     if(a==5) 
    { 
       a=0;
       send_ctr=1-send_ctr;
       CAN_send_ready(); 
        
     }
       
       PITTF_PTF0=1;  
    
  }
  
 
}
#pragma CODE_SEG DEFAULT

