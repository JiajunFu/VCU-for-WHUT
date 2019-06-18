/*********����̤������********/
uint16 ACC_1;
uint16 ACC_2;
uint16 Torque;                             //������Ť��
/*********�ƶ�Һѹ����********/
uint16 Brake_1;
uint16 Brake_2;
uint16 Brakelimit= 2048                //�ƶ���Чֵ(����ԣ�
uint8 Brake_flag;                         //�ƶ���־λ
/*********���ߴ�������********/
uint16 Var[6];                              //���ߴ�������
uint8 DES[3]={0x00,0x00,0x00}    //���ߴ����ַ���Լ����ã�


void Init_all (void);                       //��ʼ������
void brake_flag_set (void);           //�ƶ���ʶλ���ú���
void drive (void)��                      //drive����
void Priority_set (void);               //�ж����ȼ����ú���
/****************************************������*************************************/
void main (void)
{
  uint8 Start_flag=0;                                                           //��ʼ��־λ����
  uint8 Pre_flag=0;                                                             //׼����־λ����
  Init_all ( );
  Priority_set( );                                                                 //�����ж����ȼ�
  set_vector_handler (PIT0_VECTORn,brake_flag_set);           //�����ƶ��жϺ���
  enable_irq(PIT0_IRQn);                                                    //�ƶ��жϺ���ʹ��
  while (Pre_flag==0)                                                        //Ԥ��
      {
        if (gpio_get (PTB1)==1)
         {
          gpio_set (PTB4,1);
          Pre_flag=1;
         }    
      }
  while (Start_flag==0)                                                      //�жϵ�һ�ν����ʻ״̬
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
  set_vector_handler (PIT1_VECTORn,drive);                     //������ʻ�жϺ���
  enable_irq(PIT1_IRQn);                                                //��ʻ�жϺ���ʹ��
  while (1)                                                                         
  {
/**************************���ߴ���************************/







   uart_putbuff (UART4,DES,3);                                       //�������ߴ����ַ
   vcan_sendware (Var,sizeof(Var));                                 //��������
   DELAY_MS (50);
  }   
}


/**************************************�ж����ȼ�����************************************/
void Priority_set (void)
{
  NVIC_SetPriorityGrouping (NVIC_PriorityGroup_4);
  NVIC_SetPriority (PIT0_IRQn,0);
  NVIC_SetPriority (PIT1_IRQn,1);
}
/***************************************�жϺ���****************************************/
void drive (void)
{
  uint16 PedUpLimit=4095;                                               //����̤���ϼ���λ�ã����Լ����ã�
  uint16 PedDownLimit=0;                                                //����̤���¼���λ�ã����Լ����ã�
  uint16 PedMax=200;                                                     //����̤��������ֵ�����ݾ���������ã�
  ACC_1=adc_once(ADC0_DP1,ADC_12bit);
  ACC_2=2*adc_once(ADC0_DM1,ADC_12bit);
  if (ACC_1<PedDownLimit||ACC_2<PedDownLimit||ACC_1>PedUpLimit||ACC_2>PedUpLimit)    //�жϼ���̤���Ƿ񳬳�
     Torque=0;
  else if (fabs(ACC_1-ACC_2)>PedMax)                             //�ж�����Ƿ����
     Torque=0;
  else if (Brake_flag==1)
     Torque=0;
  else
     {
       if (gpio_get(PTB3)==1)                                             //�ж��Ƿ��µ�����ť
          {
            gpio_set (PTB6,0);
            gpio_set (PTB7,1);
            gpio_set (PTB21,1);                                             //��������
            gpio_set (PTB22,1);                                            //��������
            Torque=((ACC_1+ACC_2)/2-PedDownLimit)*4095/(PedUpLimit-PedDownLimit)
          }
       else
          {
            gpio_set (PTB6,1);
            gpio_set (PTB7,0);
            gpio_set (PTB21,0);                                             //��������
            gpio_set (PTB22,0);                                            //��������
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

/***************************************��ʼ������************************************/
void Init_all (void)
{
                 //��ʼ���Ƕȴ�������Һѹ������
//�Ƕȴ�����
  adc_init (ADC0_DP1);
  adc_init (ADC0_DM1);
//Һѹ������
  adc_init (ADC1_DP1);
  adc_init (ADC1_DM1);

                  //��ʼ�����ߴ���
  uart_init (UART4,9600);             
  gpio_init (PTE26,GPO,0);          
  gpio_init (PTE27,GPO,0);          
  gpio_init (PTE28,GPI,0);       

                  //��ʼ��CANͨ��
  can_init(CAN0,CAN_BAUD_20K,CAN_NORMAL,CAN_CLKSRC_OSC);

                 //���ٴ�������ʼ��
  ftm_input_init (FTM0,FTM_CH4,FTM_Rising,FTM_PS_1); 
  ftm_input_init (FTM0,FTM_CH5,FTM_Rising,FTM_PS_1); 
  ftm_input_init (FTM0,FTM_CH6,FTM_Rising,FTM_PS_1); 
  ftm_input_init (FTM0,FTM_CH7,FTM_Rising,FTM_PS_1); 

                 //��ʼ���������͵�
  gpio_init (PTB20,GPO,0);
  gpio_init (PTB21,GPO,0);  
  gpio_init (PTB22,GPO,0);  

                //��ʼ�������ź�
  gpio_init(PTB1,GPI,0);
  gpio_init(PTB2,GPI,0);
  gpio_init(PTB3,GPI,0);

               //��ʼ������ź�
  gpio_init(PTB4,GPO,0);
  gpio_init(PTB5,GPO,0);
  gpio_init(PTB6,GPO,0);
  gpio_init(PTB7,GPO,0);
  dac_init (DAC0);

              //��ʼ���жϺ���
  pit_init_ms (PIT0,20);
  pit_init_ms (PIT1,20);
}