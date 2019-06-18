/*********加速踏板输入********/
uint16 ACC_1;
uint16 ACC_2;
uint16 Torque;                             //输出最大扭矩
/*********制动液压输入********/
uint16 Brake_1;
uint16 Brake_2;
uint16 Brakelimit= 2048                //制动有效值(需调试）
uint8 Brake_flag;                         //制动标志位
/*********无线传输数据********/
uint16 Var[6];                              //无线传输数据
uint8 DES[3]={0x00,0x00,0x00}    //无线传输地址（自己设置）


void Init_all (void);                       //初始化函数
void brake_flag_set (void);           //制动标识位设置函数
void drive (void)；                      //drive函数
void Priority_set (void);               //中断优先级设置函数
/****************************************主函数*************************************/
void main (void)
{
  uint8 Start_flag=0;                                                           //开始标志位置零
  uint8 Pre_flag=0;                                                             //准备标志位置零
  Init_all ( );
  Priority_set( );                                                                 //设置中断优先级
  set_vector_handler (PIT0_VECTORn,brake_flag_set);           //设置制动中断函数
  enable_irq(PIT0_IRQn);                                                    //制动中断函数使能
  while (Pre_flag==0)                                                        //预充
      {
        if (gpio_get (PTB1)==1)
         {
          gpio_set (PTB4,1);
          Pre_flag=1;
         }    
      }
  while (Start_flag==0)                                                      //判断第一次进入待驶状态
      {
        if ((Brake_flag==1)&&(gpio_get(PTB2)==1))
         {
          gpio_set (PTB5 ,1);
          gpio_set (PTB20,1);
          DELAY_MS(3000);
          gpio_set (PTB20,0);
          Start_flag=1;
         }
      }
  set_vector_handler (PIT1_VECTORn,drive);                     //设置行驶中断函数
  enable_irq(PIT1_IRQn);                                                //行驶中断函数使能
  while (1)                                                                         
  {
/**************************无线传输************************/







   uart_putbuff (UART4,DES,3);                                       //发送无线传输地址
   vcan_sendware (Var,sizeof(Var));                                 //发送数据
   DELAY_MS (50);
  }   
}


/**************************************中断优先级设置************************************/
void Priority_set (void)
{
  NVIC_SetPriorityGrouping (NVIC_PriorityGroup_4);
  NVIC_SetPriority (PIT0_IRQn,0);
  NVIC_SetPriority (PIT1_IRQn,1);
}
/***************************************中断函数****************************************/
void drive (void)
{
  uint16 PedUpLimit=4095;                                               //加速踏板上极限位置（可自己设置）
  uint16 PedDownLimit=0;                                                //加速踏板下极限位置（可自己设置）
  uint16 PedMax=200;                                                     //加速踏板误差最大值（根据具体情况设置）
  ACC_1=adc_once(ADC0_DP1,ADC_12bit);
  ACC_2=2*adc_once(ADC0_DM1,ADC_12bit);
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
  dac_out (DAC0,Torque);
  PIT_Flag_Clear (PIT1);
}

void brake_flag_set (void)
{
  Brake_1=adc_once(ADC1_DP1,ADC_12bit);
  Brake_2=adc_once(ADC1_DM1,ADC_12bit);
  if(Brake_1>Brakelimit||Brake_2>Brakelimit)
    Brake_flag=1;
  else
    Brake_flag=0;
  PIT_Flag_Clear (PIT0);
}

/***************************************初始化函数************************************/
void Init_all (void)
{
                 //初始化角度传感器和液压传感器
//角度传感器
  adc_init (ADC0_DP1);
  adc_init (ADC0_DM1);
//液压传感器
  adc_init (ADC1_DP1);
  adc_init (ADC1_DM1);

                  //初始化无线传输
  uart_init (UART4,9600);             
  gpio_init (PTE26,GPO,0);          
  gpio_init (PTE27,GPO,0);          
  gpio_init (PTE28,GPI,0);       

                  //初始化CAN通信
  can_init(CAN0,CAN_BAUD_20K,CAN_NORMAL,CAN_CLKSRC_OSC);

                 //轮速传感器初始化
  ftm_input_init (FTM0,FTM_CH4,FTM_Rising,FTM_PS_1); 
  ftm_input_init (FTM0,FTM_CH5,FTM_Rising,FTM_PS_1); 
  ftm_input_init (FTM0,FTM_CH6,FTM_Rising,FTM_PS_1); 
  ftm_input_init (FTM0,FTM_CH7,FTM_Rising,FTM_PS_1); 

                 //初始化蜂鸣器和灯
  gpio_init (PTB20,GPO,0);
  gpio_init (PTB21,GPO,0);  
  gpio_init (PTB22,GPO,0);  

                //初始化开关信号
  gpio_init(PTB1,GPI,0);
  gpio_init(PTB2,GPI,0);
  gpio_init(PTB3,GPI,0);

               //初始化输出信号
  gpio_init(PTB4,GPO,0);
  gpio_init(PTB5,GPO,0);
  gpio_init(PTB6,GPO,0);
  gpio_init(PTB7,GPO,0);
  dac_init (DAC0);

              //初始化中断函数
  pit_init_ms (PIT0,20);
  pit_init_ms (PIT1,20);
}