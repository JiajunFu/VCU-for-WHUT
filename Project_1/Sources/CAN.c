/////////////////////////////////////////////////////////////////////////
//                                                                     //
//ע�⣺CAN���ж�Ҫ������ģʽ֮��cCAN����Ҫ��TXE                      //
/////////////////////////////////////////////////////////////////////////


#include <hidef.h>      /* common defines and macros */
#include <mc9s12XS128.h>
#include "derivative.h" 
#include "CAN.h"
#include "ADC.h"

unsigned char  j,k;
  
#define ID1                 0x8FF20EF   //VCU���ͱ���1
#define ID2                 0x8FF22EF   //VCU���ͱ���2

#define ID3                 0xCFF30F0   //����MCU����1
#define ID4                 0xCFF32F0   //����MCU����2
#define ID5                 0xCFF34F0   //����MCU����3
#define ID6                 0xCFF36F0   //����MCU����4
struct can_msg                           //���ͱ��ĵĽṹ��
{
    unsigned long id;
    char RTR;
    unsigned  char data[8];                //char  data[8];
    unsigned  char data_lenth;                  //�����ֽڳ���
    unsigned  char prty;                   //���ȼ�
} msg,tmsg,rmsg ;


unsigned long id;
unsigned char   data_lenth=8;
unsigned char   send_buf;  

unsigned  char data[8]={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

unsigned char Life ;


unsigned char send_ctr;

unsigned char VCU_1_senddata[8] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
unsigned char VCU_2_senddata[8] ={0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};


/*************************************************************/
/*                        CAN��ȡ����                        */
/*************************************************************/
void Read_Can(void)
{ if(rmsg.id==0xCFF30F0) 
 {
   TMInvCurrent= (((( unsigned int)(rmsg.data[1]))<<8)|(rmsg.data[0])); //���ĸ�ߵ���
   TMInvVoltage= (((( unsigned int)(rmsg.data[3]))<<8)|(rmsg.data[2])); //���ĸ�ߵ�ѹ
   TMInvTemp= rmsg.data[4] ;  // ���������¶�
   TMTemp= rmsg.data[5] ;     // ����¶�
   
   TM_Mode= rmsg.data[6] ;
   TMReady= TM_Mode&0x01;             // �������
   TMContModeActual= ((TM_Mode&0x06)>>1);    // �����ǰ����ģʽ
   MCU_SelfCheckPassed= ((TM_Mode&0x08)>>3); // MCU�Լ�ͨ��
   TMInvCurrent_valid=  ((TM_Mode&0x10)>>4); // ���ĸ�ߵ���ֵ��Ч��
   TMInvVoltage_valid=  ((TM_Mode&0x20)>>5); // ���ĸ�ߵ�ѹֵ��Ч��
   TMInvTemp_valid=     ((TM_Mode&0x40)>>6); // ���������¶�ֵ��Ч��
   TMTemp_valid=        ((TM_Mode&0x80)>>7); // ����¶�ֵ��Ч��
   
   Life= ((rmsg.data[7])&0x0f) ;              // MCU Message1�����ź�
 }
  else if(rmsg.id==0xCFF32F0) 
  {
   TMTq= (((unsigned int)(rmsg.data[1]))<<8)|(rmsg.data[0]);              //�����ǰת��
   TMAllowPosTqMax= (((unsigned int)(rmsg.data[3]))<<8)|(rmsg.data[2]);   //��������������ת��ֵ
   TMAllowNegTqMax= (((unsigned int)(rmsg.data[5]))<<8)|(rmsg.data[4]);   //������������ת��ֵ
  
   VCU2_BYTE_6= rmsg.data[6] ;
   TMTq_valid=  ((VCU2_BYTE_6&0x01)>>0);             // ���ת����Ч��
   TMAllowNegTqMin_valid= ((VCU2_BYTE_6&0x02)>>1);   // ��������������ת��ֵ��Ч��
   TMAllowPosTqMax_valid= ((VCU2_BYTE_6&0x04)>>2);   // ���������󷢵�ת��ֵ��Ч��
                            
   Life= ((rmsg.data[7])&0x0f) ;              // MCU Message2�����ź�
  }
  
  else if(rmsg.id==0xCFF34F0) 
 {
  TMSpd= (((unsigned int)(rmsg.data[1]))<<8)|(rmsg.data[0]);      //�����ǰת��
  TMSpd_valid= ((rmsg.data[2])&0x01) ;                            //�����ǰת��ֵ��Ч��
  
  VCU3_BYTE_3= rmsg.data[3] ;
  TMOverSpeedFlt= VCU3_BYTE_3&0x01;    // ���ٹ���
  TMUnderVoltFlt= ((VCU3_BYTE_3&0x02)>>1);    // Ƿѹ����
  TMOverVoltFlt = ((VCU3_BYTE_3&0x04)>>2);    // ��ѹ����
  TMOverLoadFlt = ((VCU3_BYTE_3&0x08)>>3);    // ���ع���
  TMResolverFlt = ((VCU3_BYTE_3&0x10)>>4);    // �������
  MCUCANCommFlt = ((VCU3_BYTE_3&0x20)>>5);    // CANͨ�Ź���
  TMInternalFlt = ((VCU3_BYTE_3&0x40)>>6);    // �ڲ�����
  TqMonitorFlt  = ((VCU3_BYTE_3&0x80)>>7);    // Ť�ؼ�ع���
  
  VCU3_BYTE_4= rmsg.data[4] ;
  TMIGBTFlt1=  VCU3_BYTE_4&0x01;            // IGBT1����
  TMIGBTFlt2=  ((VCU3_BYTE_4&0x02)>>1);            // IGBT2����
  TMIGBTFlt3=  ((VCU3_BYTE_4&0x04)>>2);            // IGBT3����
  TMHWOverCurrFlt1= ((VCU3_BYTE_4&0x08)>>3);       // Ӳ����������1
  TMHWOverCurrFlt2= ((VCU3_BYTE_4&0x10)>>4);       // Ӳ����������2
  TMPhaseACurrSensorFlt=  ((VCU3_BYTE_4&0x20)>>5); // A���������������
  TMPhaseBCurrSensorFlt=  ((VCU3_BYTE_4&0x40)>>6); // B���������������
  DischargeActFlt= ((VCU3_BYTE_4&0x80)>>7);        // �����ŵ����
  
  VCU3_BYTE_5= rmsg.data[5] ;
  TMIGBTTempSensorOCFlt=  VCU3_BYTE_5&0x01;       //IGBT�¶ȴ�������·
  MCUTempSensorOCFlt=  ((VCU3_BYTE_5&0x02)>>1);          //�������¶ȴ�������·
  TMTempSensorOCFlt=  ((VCU3_BYTE_5&0x04)>>2);           //����¶ȴ�������·
  TMIGBTTempSensorSCFlt= ((VCU3_BYTE_5&0x08)>>3);        //IGBT�¶ȴ�������·
  MCUTempSensorSCFlt= ((VCU3_BYTE_5&0x10)>>4);           //�������¶ȴ�������·
  TMTempSensorSCFlt=  ((VCU3_BYTE_5&0x20)>>5);           // ����¶ȴ�������·
  TMRotorPosCheckFlt=  ((VCU3_BYTE_5&0x40)>>6);          // ���ת��λ�ü���쳣
 
  VCU3_BYTE_6= rmsg.data[6] ;
  TMIGBTOverTempFlt=  VCU3_BYTE_6&0x01;       //IGBT�¶ȹ���
  MCUOverTempFlt=  ((VCU3_BYTE_6&0x02)>>1);          //�������¶ȹ���
  TMOverTempFlt=  ((VCU3_BYTE_6&0x04)>>2);           //����¶ȹ���
 
  Life= ((rmsg.data[7])&0x0f) ;               // MCU Message3�����ź�
 }
  
  else if(rmsg.id==0xCFF36F0) 
 {
  VCU4_BYTE_0= rmsg.data[0] ;
  M_LV_WARN=  VCU4_BYTE_0&0x01;               //���Ƿѹ����
  M_OV_WARN=  ((VCU4_BYTE_0&0x02)>>1);               //�����ѹ����
  M_PRE_CHARG_WARN=  ((VCU4_BYTE_0&0x04)>>2);        //���Ԥ������
  M_OTEMP_WARN=  ((VCU4_BYTE_0&0x08)>>3);            //������¾���
  M_OVEHICLE_WARN=  ((VCU4_BYTE_0&0x10)>>4);         //������پ���
  M_BLOCK_FAULT=  ((VCU4_BYTE_0&0x20)>>5);           //�����ת����
  BUS_OCURRENT_FAULT=  ((VCU4_BYTE_0&0x40)>>6);      //ĸ�߹�������
  MCU_OTEMP_FAULT=  ((VCU4_BYTE_0&0x80)>>7);         //MCU���¾���
  
  VCU4_BYTE_1= rmsg.data[1] ;
  IGBT_OVTEMP_WARN=  VCU4_BYTE_1&0x01;                //IGBT���¾���
  LV_BETT_LV_FLT=    ((VCU4_BYTE_1&0x02)>>1);                //��ѹ��ص�ѹ�͹���
  BUS_CURRENT_SENSOR_Flt=  ((VCU4_BYTE_1&0x04)>>2);          //ĸ�ߵ�����������·����
  BUS_CURRENT_SENSOR_OPEN=  ((VCU4_BYTE_1&0x08)>>3);         //ĸ�ߵ�����������·    
  BUS_CURRENT_SENSOR_OVRANGE_Flt = ((VCU4_BYTE_1&0x10)>>4);  // ĸ�ߵ����������ź�ƫ��ֵ����������Χ
  BUS_VOLTAGE_SENSOR_OPEN_Flt=  ((VCU4_BYTE_1&0x20)>>5);     // ĸ�ߵ�ѹ��������·
 }
  else 
    ;       
}

/*************************************************************/
/*                        CAN��ʼ��                          */
/*************************************************************/
void CAN_Init(void) 
{
  if(CAN0CTL0_INITRQ==0)       // ��ѯ�Ƿ�����ʼ��״̬   
    CAN0CTL0_INITRQ =1;        // �����ʼ��״̬

  while (CAN0CTL1_INITAK==0);  // �ȴ������ʼ��״̬

  CAN0BTR0_SJW = 0;            //����ͬ��
  CAN0BTR0_BRP = 7;            //���ò�����  
  CAN0BTR1 = 0x1c;             //����ʱ��1��ʱ��2��Tq���� ,������Ϊ250kb/s

  // �ر��˲���                                  
  CAN0IDMR0 = 0xFF;
  CAN0IDMR1 = 0xFF;
  CAN0IDMR2 = 0xFF;
  CAN0IDMR3 = 0xFF;
  CAN0IDMR4 = 0xFF;
  CAN0IDMR5 = 0xFF;
  CAN0IDMR6 = 0xFF;
  CAN0IDMR7 = 0xFF; 

  CAN0CTL1 = 0xc0;             //ʹ��MSCANģ��,����Ϊһ������ģʽ��ʹ������ʱ��Դ 

  CAN0CTL0 = 0x00;             //����һ��ģʽ����

  while(CAN0CTL1_INITAK);      //�ȴ��ص�һ������ģʽ

  while(CAN0CTL0_SYNCH==0);    //�ȴ�����ʱ��ͬ��

  CAN0RIER_RXFIE = 0;          //0:��ֹ�����ж�; 1:ʹ�ܽ����ж� ,���ĳɹ����մ����ж�
}

/*************************************************************/
/*                       CAN����                             */
/*************************************************************/
unsigned char MSCAN0SendMsg(struct can_msg tmsg)
{
  unsigned char send_buf,t,sp;
 // unsigned  char data_lenth=8;
  // ������ݳ���
  if(tmsg.data_lenth > 8)
    return(FALSE); 

  // �������ʱ��
  if(CAN0CTL0_SYNCH==0)
    return(FALSE);

  send_buf = 0;
  do
  {
    // Ѱ�ҿ��еĻ�����
    CAN0TBSEL=CAN0TFLG;
    send_buf=CAN0TBSEL;
  }
  
  while(!send_buf); 
  
 
  if(tmsg.RTR)
    // RTR = ����
    CAN0TXIDR3 |= 0x01;
    
    // CANд�����ݳ��ȣ�8�ֽ�  
   CAN0TXDLR=tmsg.data_lenth; 
   
    // CANд������
   for(sp = 0; sp < data_lenth; sp++)
     *((&CAN0TXDSR0)+sp) = tmsg.data[sp]; 
  
  
  /***CANд��ID***/
  CAN0TXIDR0 = (unsigned char)(tmsg.id>>21);
  CAN0TXIDR1 = (((unsigned char)(tmsg.id>>13))&0xe0)|0x18|(((unsigned char)(tmsg.id>>15))&0x07);
  CAN0TXIDR2 = (unsigned char)(tmsg.id>>7);
  CAN0TXIDR3 = ((unsigned char)(tmsg.id<<1));  
   
  // д�����ȼ�
  //CAN0TXTBPR = msg.prty;
  
  // �� TXx ��־ (������׼������)
  CAN0TFLG = send_buf;
  
  if(CAN0TFLG)
     t=1;
   else
     t=0;
    return(t);       //t��Ϊ����ֵ����ΪCAN_send_ready()�������жϱ����Ƿ��ͳɹ�������
  
} 


/*************************************************************/
/*                       CAN����                            */
/*************************************************************/
unsigned char MSCAN0GetMsg(struct can_msg rmsg)
{
  unsigned char tp;

  // �����ձ�־
  if(!(CAN0RFLG_RXF))
    return(FALSE);
  
  // ��� CANЭ�鱨��ģʽ ��һ��/��չ�� ��ʶ��
  if(CAN0RXIDR1_IDE)
    // IDE = Recessive (Extended Mode)
    return(FALSE);
                     
  rmsg.id = (((unsigned long)(CAN0RXIDR0))<<21)|(((unsigned long)(CAN0RXIDR1&0xe0))<<13)| \
     (((unsigned long)(CAN0RXIDR1&0x07))<<15)|(((unsigned long)(CAN0RXIDR2))<<7)| \
     (((unsigned long)(CAN0RXIDR3&0xfe))>>1);
  if(CAN0RXIDR1&0x10)
    rmsg.RTR = TRUE;
  else
    rmsg.RTR = FALSE;
  
  // ��ȡ���ݳ��� 
  rmsg.data_lenth = CAN0RXDLR;
  
  // ��ȡ����
  for(tp = 0; tp < rmsg.data_lenth; tp++)
    rmsg.data[tp] = *((&CAN0RXDSR0)+tp);

  // �� RXF ��־λ (������׼������)
  CAN0RFLG = 0x01;


  Read_Can();                  //��ȡCAN���ݣ����CAN�ж��в��ܹ��������ձ��ģ�����MSCAN0GetMsg()
                                 //�н��ձ���
}

/*************************************************************/
/*                       CAN��������׼��                     */
/*************************************************************/     
void CAN_send_ready( void)

{  //�������     25/55kW 	70/200Nm	3000/9500	320(���ѹ)	265-370(��ѹ��Χ)

   //��VCU1��VCU2���и�ֵ ,���жϳ����﷢��VCU1��VCU2
    /************** �������������ͱ���VCU1****************/
  
 
    VCU_1_senddata[0]=(0x65)|(TMEnable<<2) |(TMFaultReset<<5);            //0x65������Ϊ01100101
    //֤�������ⲿ����ACC�󣬿��Խ�main�����е�ACCֵ����CAN.c�����е�VCU_1_senddata[0]   
    // 01000101��Ϊ0x45
  /*
  
     Bit0	�������ģʽ����TMContMode��	0 : ����ģʽ��STAND BY MODE��
     Bit1                                 1 : ת�ؿ���ģʽ��TQ CTL MODE��
                                          2 : ת�ٿ���ģʽ��SPD CTL MODE��
                                          3 : ���У�LimpHome��
   		
     Bit2	���ʹ�ܣ�TMEnable��          	1: ���ʹ��
                                          0:�����ʹ��
     Bit3	��������ŵ�����TMActiveDischarge��  	1:�����ŵ�ʹ��
                                                  0:�����ŵ��ʹ��
     Bit5	���������������TMFaultReset��      	1:����MCU���ù���       �е�û���!!!
                                                  0:ֹͣ����MCU���ù���
                                          
     Bit6	���ת��������Ч�ԣ�TMTqReq_valid��	  1 : �ź���Ч
                                                0 : �ź���Ч
     Bit7	���ת��������Ч�ԣ�TMSpdReq_valid��	1 : �ź���Ч
                                                0 : �ź���Ч
   */
 
  VCU_1_senddata[1]=(unsigned char)(((Torque+3000)*10)&0x00ff);       // Ĭ��Ϊת�ص�8λ, �ߵ��ֽ���Э����û�ж��壬������ȷ�� 
  VCU_1_senddata[2]=(unsigned char)(((Torque+3000)*10)>>8);           // Ĭ��Ϊת�ظ�8λ, �����������źţ�Torque��ֵ
  //BYTE 1,BYTE 2 ,���ת������TMTqReq��  from Torque�ź�
  
  /*�ź�����ֵ���߼�ֵ��ת����ʽΪ���ź�����ֵ = �������ӡ��ź��߼�ֵ+ ƫ����
 	�������ת���źŵı�������Ϊ0.25��ƫ����Ϊ0����������ת����ϵ��
 	2000rpm = 8000��1F40h�� * 0.25 + 0 
  */
  
  VCU_1_senddata[3]=0;
  VCU_1_senddata[4]=0;       //BYTE3	BYTE4   ���ת������TMSpdReq��
  	                         //ƫ����-3000����������1
                             //���ڲ���ת�����ģʽ���Ͳ��ؿ���ת��
                             
  VCU_1_senddata[5]=0x00;                        
  VCU_1_senddata[6]=0x00   ; 
  //BYTE5	 BYTE6 ���������ֽ���Ч
 
  VCU_1_senddata[7]=Life;       
  //BYTE7	VCU Message1 �����ź�
 
  
  /************** �������������ͱ���VCU2****************/
  VCU_2_senddata[0]=0       ;
  VCU_2_senddata[1]=0       ;               //��Ҫ��ϸ���ǣ�����
  //BYTE0	 BYTE1  ���ת�ر仯����ֵ��TMTqSlewRate��
   
  VCU_2_senddata[2]=(unsigned char)(((TMAllowPosTqMax+3000)*10)&0x00ff);     
  VCU_2_senddata[3]=(unsigned char)(((TMAllowPosTqMax+3000)*10)>>8);                   
  //BYTE2	BYTE3  ����ת����ֵ��PosTqLimit�� ������������ת�أ���ֵ
    
  VCU_2_senddata[4]=(unsigned char)(((0+3000)*10)&0x00ff);     
  VCU_2_senddata[5]=(unsigned char)(((0+3000)*10)>>8);                  
  //BYTE4 BYTE5	����ת����ֵ��NegTqLimit�� �������Ƹ���ת�أ���ֵ
  //�������ܹ�������߷�����ʻ��ֻ��һ���������ﷴ��ת����Ϊ0
  
  VCU_2_senddata[6]=0x00       ; //BYTE6��Ч�ֽ�
  
  VCU_2_senddata[7]=Life       ;

  //BYTE7	Bit0  	Bit1   	Bit2  Bit3   VCU Message2 �����ź�
  
  
 
   if(send_ctr==1)               //����VCU1
      { msg.id = ID1;  
        msg.data_lenth=8;
         for(j=0;j<data_lenth;j++)
        {
          msg.data[j] = VCU_1_senddata[j];
        }
   
        if(!(MSCAN0SendMsg(msg)))   //���͹��̳��ִ���
           ;
        
         else
          ;
      } 
      
     
    else if (send_ctr==0)       //����VCU2
      { msg.id = ID2; 
        msg.data_lenth=8;
    
        for(k=0;k<data_lenth;k++)
        {
            msg.data[k] = VCU_2_senddata[k];
         }    
     
       if(!MSCAN0SendMsg(msg))     //���͹��̳��ִ���       
          ;
      else
          ;
      }
     else
      ; 
          
}

/*************************************************************/
/*                       CAN�жϴ���                         */
/*************************************************************/

#pragma CODE_SEG NON_BANKED
interrupt VectorNumber_Vcan0rx void CANReceive_ISR (void)                 // CAN�����ж�
{ 
  DisableInterrupts;
  
  
  Read_Can();            //��ȡCAN����
  CAN0RFLG_RXF=0;        //���ջ���Ĵ�������,�ͷŽ��ջ�����
  
  
  EnableInterrupts; 
}

/*interrupt VectorNumber_Vcan0err void CANErr_ISR (void)                  // CAN����֡�����ж�
{
  DisableInterrupts;
  
  CAN_Init();
  EnableInterrupts;
 
}
*/
#pragma CODE_SEG DEFAULT
