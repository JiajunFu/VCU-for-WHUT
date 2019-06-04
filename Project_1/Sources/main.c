#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "IO.h"
#include "PLL.h"
#include "ECT.h"
#include "ADC.h"
#include "CAN.h"
#include "PIT.h"
#define PedMax 0.1           
#define PedUpLimit 5  		//̤���г̵�ѹ��Χ���
unsigned int ACC_1;      //���Ŵ�����1
unsigned int ACC_2;      //���Ŵ�����2
unsigned int Torque;     //ת��
void Drive(void);
/***����������***/
unsigned char on_fire ;   //ON�𳣱ա�
unsigned char brake;      //�ƶ�
unsigned char start;      //�������㶯�򳣱�

/***���������***/
unsigned char Pre_charge; //���Ԥ���
unsigned char charge;     //�������̵���
unsigned char buzz;       //����   
           
 /******************************VCU1��������*******************************/
  unsigned char TMEnable;    //���ʹ�� 1: ���ʹ�� �� 0:�����ʹ��

  unsigned char TMFaultReset; //���������������   1:����MCU���ù��� �� 0: ֹͣ����MCU���ù���
 
 
 /******************************MCU1��������*******************************/
   
   
   unsigned int  TMInvCurrent; //���ĸ�ߵ���
   unsigned int  TMInvVoltage; //���ĸ�ߵ�ѹ
   unsigned int  TMInvTemp  ;  //���������¶�
   unsigned int  TMTemp;       //����¶�
   unsigned int  TM_Mode ;     //MCU1���߸��ֽڣ����ģʽ
  
   char              TMReady;             // �������
   char              TMContModeActual;    // �����ǰ����ģʽ
   char              MCU_SelfCheckPassed; // MCU�Լ�ͨ��
   char              TMInvCurrent_valid;  // ���ĸ�ߵ���ֵ��Ч��
   char              TMInvVoltage_valid;  // ���ĸ�ߵ�ѹֵ��Ч��
   char              TMInvTemp_valid;     // ���������¶�ֵ��Ч��
   char              TMTemp_valid;        // ����¶�ֵ��Ч��
   
 /******************************MCU2��������*******************************/
   unsigned int TMTq;                //�����ǰת��
   unsigned int TMAllowPosTqMax;     //��������������ת��ֵ
   unsigned int TMAllowNegTqMax;     //������������ת��ֵ
   unsigned int VCU2_BYTE_6;         //MCU2���߸��ֽڣ����ģʽ
 
   char             TMTq_valid;              // ���ת����Ч��
   char             TMAllowNegTqMin_valid;   // ��������������ת��ֵ��Ч��
   char             TMAllowPosTqMax_valid;   // ���������󷢵�ת��ֵ��Ч��
 
 /******************************MCU3��������*******************************/
 
   unsigned int TMSpd;                 //�����ǰת��
   unsigned int TMSpd_valid;           //�����ǰת��ֵ��Ч��
   unsigned int VCU3_BYTE_3 ;          //MCU3���ĸ��ֽڣ����ģʽ
  
   char             TMOverSpeedFlt;        // ���ٹ���
   char             TMUnderVoltFlt;        // Ƿѹ����
   char             TMOverVoltFlt;         // ��ѹ����
   char             TMOverLoadFlt ;        // ���ع���
   char             TMResolverFlt ;        // �������
   char             MCUCANCommFlt ;        // CANͨ�Ź���
   char             TMInternalFlt;         // �ڲ�����
   char             TqMonitorFlt  ;        // Ť�ؼ�ع���
  
   unsigned int VCU3_BYTE_4 ;          //MCU3������ֽڣ����ģʽ
               
   char             TMIGBTFlt1 ;              // IGBT1����
   char             TMIGBTFlt2 ;              // IGBT2����
   char             TMIGBTFlt3 ;              // IGBT3����
   char             TMHWOverCurrFlt1 ;        // Ӳ����������1
   char             TMHWOverCurrFlt2 ;        // Ӳ����������2
   char             TMPhaseACurrSensorFlt ;   // A���������������
   char             TMPhaseBCurrSensorFlt ;   // B���������������
   char             DischargeActFlt ;         // �����ŵ����
  
   unsigned int VCU3_BYTE_5 ;                 //MCU3�������ֽڣ����ģʽ
  
   char             TMIGBTTempSensorOCFlt;        //IGBT�¶ȴ�������·
   char             MCUTempSensorOCFlt;           //�������¶ȴ�������·
   char             TMTempSensorOCFlt;            //����¶ȴ�������·
   char             TMIGBTTempSensorSCFlt;        //IGBT�¶ȴ�������·
   char             MCUTempSensorSCFlt;           //�������¶ȴ�������·
   char             TMTempSensorSCFlt;            //����¶ȴ�������·
   char             TMRotorPosCheckFlt;           //���ת��λ�ü���쳣
 
   unsigned int VCU3_BYTE_6;                 //MCU3���߸��ֽڣ����ģʽ
  
   char             TMIGBTOverTempFlt;           //IGBT�¶ȹ���
   char             MCUOverTempFlt;              //�������¶ȹ���
   char             TMOverTempFlt;               //����¶ȹ���
   char             CommunicationFltl;            //ͨѶ����1
   char             CommunicationFlt2;            //ͨѶ����2
 
 /******************************MCU4��������*******************************/
   unsigned int VCU4_BYTE_0 ;                //MCU4��һ���ֽڣ����ģʽ
 
   char             M_LV_WARN;                   //���Ƿѹ����
   char             M_OV_WARN;                   //�����ѹ����
   char             M_PRE_CHARG_WARN;            //���Ԥ������
   char             M_OTEMP_WARN;                //������¾���
   char             M_OVEHICLE_WARN;             //������پ���
   char             M_BLOCK_FAULT;               //�����ת����
   char             BUS_OCURRENT_FAULT;          //ĸ�߹�������
   char             MCU_OTEMP_FAULT;             //MCU���¾���
  
   unsigned int VCU4_BYTE_1  ;                    // MCU4�ڶ����ֽڣ����ģʽ
  
   char             IGBT_OVTEMP_WARN ;                // IGBT���¾���
   char             LV_BETT_LV_FLT ;                  // ��ѹ��ص�ѹ�͹���
   char             BUS_CURRENT_SENSOR_Flt ;          // ĸ�ߵ�����������·����
   char             BUS_CURRENT_SENSOR_OPEN ;         // ĸ�ߵ�����������·    
   char             BUS_CURRENT_SENSOR_OVRANGE_Flt ;  // ĸ�ߵ����������ź�ƫ��ֵ����������Χ
   char             BUS_VOLTAGE_SENSOR_OPEN_Flt ;     // ĸ�ߵ�ѹ��������·



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
/*                         ������                            */
/*************************************************************/
 void main(void)
{  
   unsigned char start_flg=0;
   DisableInterrupts;
    Init_all() ;
 
   EnableInterrupts; 
   on_fire = SIN_1;              //�����ⲿ�����ź�,ON��
   start   = SIN_2;              //����
   brake   = SIN_3;              //�ƶ�
  
  
  
   W_1 = charge;            //��������źţ������ⲿ���̵���
   W_2 = buzz;                  //���� 
   W_3 = Pre_charge;            //���Ԥ���
   
   
   ACC_1= ((unsigned int)(AD_value[1])<<8)|(AD_value[0]);
   ACC_2= ((unsigned int)(AD_value[3])<<8)|(AD_value[2]);  
   
   	if(on_fire)     // on_fireΪ�㶯���� 
		{
			   charge=0;                             // ���̵����Ͽ�
                Pre_charge=1;                         // ���Ԥ��磬�Ե���𵽱�������        
                delay1ms( 700) ;                          // ��ʱ0.7��
                 Pre_charge=0;                             // �ر�Ԥ���    
                 charge=1;           
		}
       
    	while(1)
	{
     	if((brake&start)|start_flg)
			{
				if((brake&start)&&(start_flg==0))  //�жϵ�һ�ν����ʻ״̬���ƶ�̤�壬on������
					{
						buzz=1;
						delay1ms(3000);
						buzz=0;
						start_flg=1;
					}
				Drive();
			}
			else
			{
				Torque=0;
			}
		}
	


}

/*************************************** subfunction ***************************************/

void Drive(void)
{
    unsigned char PedWait_flg=0;
    unsigned int  Trq_re_temp=0;
	if((ACC_1-ACC_2)>PedMax)  //���������ϼ�⡣2���������ź�֮���ֵ����һ��ֵ����Ϊ�й���
	{
	Torque=0;
	}
	else if(brake|PedWait_flg)  //
	{
	Torque=0;
		if(ACC_1>(0.25*PedUpLimit))//�����ſ��Ȳȵ�25%���£��Ҳ����ƶ�̤�壬���������Ų��������ת��
		{
			PedWait_flg=1;
		}
		if((ACC_1<0.05*PedUpLimit)&&(PedWait_flg==1))
		{
			PedWait_flg=0;
		}
	}
	else 
	{	
		Trq_re_temp=(unsigned int)(ACC_1*40);//�����С��Ҫ���ԡ�Ť�ط�Χ0-200N
		if(Trq_re_temp>200)
		{
			Trq_re_temp=200;
		}
		if(Trq_re_temp<=0)
		{
			Trq_re_temp=0;
		}
		Torque=Trq_re_temp;
	}
	
}



 
 




