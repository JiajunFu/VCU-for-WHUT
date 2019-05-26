#include <hidef.h>
#include "derivative.h" /* include peripheral declarations */
#include "CAN.h"

/*************************************************************/
/*                        ��ʼ��CAN0                         */
/*************************************************************/
void INIT_CAN0(void) 
{
  if(CAN0CTL0_INITRQ==0)      // ��ѯ�Ƿ�����ʼ��״̬   
    CAN0CTL0_INITRQ =1;        // �����ʼ��״̬

  while (CAN0CTL1_INITAK==0);  //�ȴ������ʼ��״̬

  CAN0BTR0_SJW = 0;            //����ͬ��
  CAN0BTR0_BRP = 7;            //���ò�����  
  CAN0BTR1 = 0x1c;       //����ʱ��1��ʱ��2��Tq���� ,����Ƶ��Ϊ250kb/s

// �ر��˲���                                  
  CAN0IDMR0 = 0xFF;
  CAN0IDMR1 = 0xFF;
  CAN0IDMR2 = 0xFF;
  CAN0IDMR3 = 0xFF;
  CAN0IDMR4 = 0xFF;
  CAN0IDMR5 = 0xFF;
  CAN0IDMR6 = 0xFF;
  CAN0IDMR7 = 0xFF; 

  CAN0CTL1 = 0xC0;             //ʹ��MSCANģ��,����Ϊһ������ģʽ��ʹ������ʱ��Դ 

  CAN0CTL0 = 0x00;             //����һ��ģʽ����

  while(CAN0CTL1_INITAK);      //�ȴ��ص�һ������ģʽ

  while(CAN0CTL0_SYNCH==0);    //�ȴ�����ʱ��ͬ��

  CAN0RIER_RXFIE = 0;          //��ֹ�����ж�
}

/*************************************************************/
/*                       CAN0����                            */
/*************************************************************/
Bool MSCAN0SendMsg(struct can_msg msg)
{
  unsigned char send_buf, sp;
  
  // ������ݳ���
  if(msg.len > 8)
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
  
  // д���ʶ��
  CAN0TXIDR0 = (unsigned char)(msg.id>>3);
  CAN0TXIDR1 = (unsigned char)(msg.id<<5);
  
  if(msg.RTR)
    // RTR = ����
    CAN0TXIDR1 |= 0x10;
    
  // д������
  for(sp = 0; sp < msg.len; sp++)
    *((&CAN0TXDSR0)+sp) = msg.data[sp];
    
  // д�����ݳ���
  CAN0TXDLR = msg.len; 
  
  // д�����ȼ�
  CAN0TXTBPR = msg.prty;
  
  // �� TXx ��־ (������׼������)
  CAN0TFLG = send_buf;
  
  return(TRUE);
  
}

/*************************************************************/
/*                       CAN0����                            */
/*************************************************************/
Bool MSCAN0GetMsg(struct can_msg *msg)
{
  unsigned char sp2;

  // �����ձ�־
  if(!(CAN0RFLG_RXF))
    return(FALSE);
  
  // ��� CANЭ�鱨��ģʽ ��һ��/��չ�� ��ʶ��
  if(CAN0RXIDR1_IDE)
    // IDE = Recessive (Extended Mode)
    return(FALSE);

  // ����ʶ��
  msg->id = (unsigned int)(CAN0RXIDR0<<3) | 
            (unsigned char)(CAN0RXIDR1>>5);
  
  if(CAN0RXIDR1&0x10)
    msg->RTR = TRUE;
  else
    msg->RTR = FALSE;
  
  // ��ȡ���ݳ��� 
  msg->len = CAN0RXDLR;
  
  // ��ȡ����
  for(sp2 = 0; sp2 < msg->len; sp2++)
    msg->data[sp2] = *((&CAN0RXDSR0)+sp2);

  // �� RXF ��־λ (������׼������)
  CAN0RFLG = 0x01;

  return(TRUE);
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

