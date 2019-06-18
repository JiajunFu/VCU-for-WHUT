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
        if (gpio_get(PTB1)==1)
         {
          gpio_set(PTB4,1);
          Pre_flag=1;
         }    
      }
  while (Start_flag==0)                                                      //判断第一次进入待驶状态
      {
        if (gpio_get(PTB2)==1)
         {
          gpio_set (PTB5 ,1);
          gpio_set (PTB20,1);
          delay1ms(3000);
          gpio_set (PTB20,0);
          Start_flag=1;
         }
      }
  while(1) {
          	if(start_flg)
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
  uint16 PedMax=200;                                                     //加速踏板误差最大值（根据具体情况设置）
  if (ACC_1<PedDownLimit||ACC_2<PedDownLimit||ACC_1>PedUpLimit||ACC_2>PedUpLimit)    //判断加速踏板是否超程
     Torque=0;
  else if (fabs(ACC_1-ACC_2)>PedMax)                             //判断误差是否过大
     Torque=0;
  else if (Brake_flag==1)
     Torque=0;
  else
     {
       if (gpio_get(PTB3)==1)                                             //判断是否按下倒车按钮
          {
            gpio_set (PTB6,0);
            gpio_set (PTB7,1);
            gpio_set (PTB21,1);                                             //蜂鸣器响
            gpio_set (PTB22,1);                                            //倒车灯亮
            Torque=((ACC_1+ACC_2)/2-PedDownLimit)*4095/(PedUpLimit-PedDownLimit)
          }
       else
          {
            gpio_set (PTB6,1);
            gpio_set (PTB7,0);
            gpio_set (PTB21,0);                                             //蜂鸣器关
            gpio_set (PTB22,0);                                            //倒车灯灭
            Torque=((ACC_1+ACC_2)/2-PedDownLimit)*4095/(PedUpLimit-PedDownLimit)
          }
     }
      /* PWM输出torque值给电机
}

