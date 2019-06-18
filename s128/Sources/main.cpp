#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "lib.h"
typedef unsigned char uint8;
/*********加速踏板输入********/
unsigned int ACC_1;      //油门传感器1
unsigned int ACC_2;      //油门传感器2
unsigned int Torque;     //转矩
void Init_all (void); 
void drive (void);        



/*************************************************************/
/*                     初始化函数                            */
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
/*                         延时函数                          */
/*************************************************************/
void delay1ms(unsigned int n) 
{
    unsigned int i;
    for(i=0;i<n;i++) 
    {
        TFLG1_C0F = 1;              // 清除标志位
        TC0 = TCNT + 250;           // 设置输出比较时间为1ms   4us*250 //需要将n设置为3000，对应鸣笛3秒
        while(TFLG1_C0F == 0);      // 等待，直到发生输出比较事件
    }
}

/*************************************************************/
/*                         主函数                            */
/*************************************************************/
void main(void) {
uint8 Start_flag=0;                                                           //开始标志位置零
  uint8 Pre_flag=0;                                                             //准备标志位置零
  Init_all();                                                               //设置中断优先级
  while (Pre_flag==0)                                                        //预充
      {
        if (GPIO_Get(S1IN,PRT,2)==1)
         {
          GPIO_Set(W4,PRT,3,1);
          Pre_flag=1;
         }    
      }
  while (Start_flag==0)                                                      //判断第一次进入待驶状态
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
/*                         驾驶函数                          */
/*************************************************************/
void drive (void)
{
  uint16 PedUpLimit=4095;                                               //加速踏板上极限位置（可自己设置）
  uint16 PedDownLimit=0;                                                //加速踏板下极限位置（可自己设置）
  uint16 PedMax=200; 
   ACC_1= ((unsigned int)(AD_value[1]);
   ACC_2= ((unsigned int)(AD_value[2]);                                                      //加速踏板误差最大值（根据具体情况设置）
  if (ACC_1<PedDownLimit||ACC_2<PedDownLimit||ACC_1>PedUpLimit||ACC_2>PedUpLimit)    //判断加速踏板是否超程
     Torque=0;
  else if (fabs(ACC_1-ACC_2)>PedMax)                             //判断误差是否过大
     Torque=0;
  else if (Brake_flag==1)
     Torque=0;
  else
     {
       if (GPIO_Get(S3IN,PRT,0)==1)                                             //判断是否按下倒车按钮
          {
            GPIO_Set(W6,PRT,5,0);
            GPIO_Set(W7,PRT,6,1);
            GPIO_Set(W3,PRT,2,1);                                            //蜂鸣器响
            GPIO_Set(W2,PRT,1,1);;                                            //倒车灯亮
            Torque=((ACC_1+ACC_2)/2-PedDownLimit)*4095/(PedUpLimit-PedDownLimit)
          }
       else
          {
            GPIO_Set(W6,PRT,5,1);
            GPIO_Set(W7,PRT,6,0);
            GPIO_Set(W3,PRT,2,0);                                            //蜂鸣器响
            GPIO_Set(W2,PRT,1,0);;                                            //倒车灯灭
            Torque=((ACC_1+ACC_2)/2-PedDownLimit)*4095/(PedUpLimit-PedDownLimit)
          }
     }
     
      /* PWM输出torque值给电机
}

