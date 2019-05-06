#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include"PIT.h"
#include"CAN.h"
#include"ADC.h"
unsigned char     a=0 ,b=0;

/*************************************************************/
/*                     PIT初始化                             */
/*************************************************************/
/* void PIT_Init()
 {
     PITCFLMT=0;       //不使能周期中断定时器
     PITCE_PCE0=1;     //第0通道计数器工作 
     PITMUX_PMUX0=0;   //第0通道使用微计数器0
     PITMTLD0=249;     //为0通道8位计数器赋值
     PITLD0=63;     //为0通道16位计数器赋值   //(249+1)*(127+1)=32000个总线周期=2ms,每隔2ms中断一次
     
     PITINTE_PINTE0=1; //0通道定时器定时中断被使能
     PITCFLMT=0X80;    //使能周期中断定时器
     
 }  */

/*************************************************************/
/*                     PIT初始化                             */
/*************************************************************/
 void PIT_Init()
 {
 
   PITMTLD0=249;     //为0通道8位计数器赋值
   PITLD0=255;       //为0通道16位计数器赋值   
                     //(249+1)*(1279+1)=320000个总线周期,10ms,
                     //报文1和2之间实际时间为20/3ms，报文1或2时间为20ms  ,255对应2ms
   PITMUX_PMUX0=0;   //第0通道使用微计数器0
   PITCE_PCE0=1;     //第0通道计数器工作 
   PITCFLMT=0X80;    //使能周期中断定时器
   PITINTE_PINTE0=1; //0通道定时器定时中断被使能
 }
/*************************************************************/
/*                     PIT定时中断                           */
/*************************************************************/ 
#pragma CODE_SEG __NEAR_SEG NON_BANKED
interrupt VectorNumber_Vpit0 void PIT_66U(void) 

{  

          b++;
  if(((PITTF_PTF0==1)&&(b==1)))      //2ms采集一次模拟量 ，ACC_1或者ACC_2
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

