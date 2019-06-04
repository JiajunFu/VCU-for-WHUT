#ifndef _CAN_H_

#define _CAN_H_


extern void CAN_Init(void);
extern unsigned char send_ctr;
extern unsigned char send_buf,t;
extern unsigned char  MSCAN0SendMsg(struct can_msg tmsg);
extern unsigned char  MSCAN0GetMsg(struct can_msg rmsg);

extern void Read_Can(void);
extern unsigned char VCU_1_senddata[8] ;
extern unsigned char VCU_2_senddata[8] ;

extern void CAN_send_ready(void) ;
/* struct can_msg                           //���ͱ��ĵĽṹ��
{
    unsigned long id;
     char RTR;
    unsigned  char data[8];                //char  data[8];
    unsigned  char data_lenth;                  //�����ֽڳ���
    unsigned  char prty;                   //���ȼ�
};
  */
 
 /******************************VCU1��������*******************************/
 extern unsigned char TMEnable ;                           //���ʹ�� 1: ���ʹ�� �� 0:�����ʹ��

 extern unsigned char TMFaultReset;                       //���������������   1:����MCU���ù��� �� 0: ֹͣ����MCU���ù���

 
 /******************************MCU1��������*******************************/
  extern unsigned int  TMInvCurrent; //���ĸ�ߵ���
  extern unsigned int  TMInvVoltage; //���ĸ�ߵ�ѹ
  extern unsigned int  TMInvTemp  ;  //���������¶�
  extern unsigned int  TMTemp;       //����¶�
  extern unsigned int  TM_Mode ;     //MCU1���߸��ֽڣ����ģʽ
  
  extern char              TMReady;             // �������
  extern char              TMContModeActual;    // �����ǰ����ģʽ
  extern char              MCU_SelfCheckPassed; // MCU�Լ�ͨ��
  extern char              TMInvCurrent_valid;  // ���ĸ�ߵ���ֵ��Ч��
  extern char              TMInvVoltage_valid;  // ���ĸ�ߵ�ѹֵ��Ч��
  extern char              TMInvTemp_valid;     // ���������¶�ֵ��Ч��
  extern char              TMTemp_valid;        // ����¶�ֵ��Ч��
   
 /******************************MCU2��������*******************************/
  extern unsigned int TMTq;                //�����ǰת��
  extern unsigned int TMAllowPosTqMax;     //��������������ת��ֵ
  extern unsigned int TMAllowNegTqMax;     //������������ת��ֵ
  extern unsigned int VCU2_BYTE_6;         //MCU2���߸��ֽڣ����ģʽ
 
  extern char             TMTq_valid;              // ���ת����Ч��
  extern char             TMAllowNegTqMin_valid;   // ��������������ת��ֵ��Ч��
  extern char             TMAllowPosTqMax_valid;   // ���������󷢵�ת��ֵ��Ч��
 
 /******************************MCU3��������*******************************/
 
  extern unsigned int TMSpd;                 //�����ǰת��
  extern unsigned int TMSpd_valid;           //�����ǰת��ֵ��Ч��
  extern unsigned int VCU3_BYTE_3 ;          //MCU3���ĸ��ֽڣ����ģʽ
  
  extern char             TMOverSpeedFlt;        // ���ٹ���
  extern char             TMUnderVoltFlt;        // Ƿѹ����
  extern char             TMOverVoltFlt;         // ��ѹ����
  extern char             TMOverLoadFlt ;        // ���ع���
  extern char             TMResolverFlt ;        // �������
  extern char             MCUCANCommFlt ;        // CANͨ�Ź���
  extern char             TMInternalFlt;         // �ڲ�����
  extern char             TqMonitorFlt  ;        // Ť�ؼ�ع���
  
  extern unsigned int VCU3_BYTE_4 ;          //MCU3������ֽڣ����ģʽ
               
  extern char             TMIGBTFlt1 ;              // IGBT1����
  extern char             TMIGBTFlt2 ;              // IGBT2����
  extern char             TMIGBTFlt3 ;              // IGBT3����
  extern char             TMHWOverCurrFlt1 ;        // Ӳ����������1
  extern char             TMHWOverCurrFlt2 ;        // Ӳ����������2
  extern char             TMPhaseACurrSensorFlt ;   // A���������������
  extern char             TMPhaseBCurrSensorFlt ;   // B���������������
  extern char             DischargeActFlt ;         // �����ŵ����
  
  extern unsigned int VCU3_BYTE_5 ;                 //MCU3�������ֽڣ����ģʽ
  
  extern char             TMIGBTTempSensorOCFlt;        //IGBT�¶ȴ�������·
  extern char             MCUTempSensorOCFlt;           //�������¶ȴ�������·
  extern char             TMTempSensorOCFlt;            //����¶ȴ�������·
  extern char             TMIGBTTempSensorSCFlt;        //IGBT�¶ȴ�������·
  extern char             MCUTempSensorSCFlt;           //�������¶ȴ�������·
  extern char             TMTempSensorSCFlt;            //����¶ȴ�������·
  extern char             TMRotorPosCheckFlt;           //���ת��λ�ü���쳣
 
  extern unsigned int VCU3_BYTE_6;                 //MCU3���߸��ֽڣ����ģʽ
  
  extern char             TMIGBTOverTempFlt;           //IGBT�¶ȹ���
  extern char             MCUOverTempFlt;              //�������¶ȹ���
  extern char             TMOverTempFlt;               //����¶ȹ���
  extern char             CommunicationFltl;            //ͨѶ����1
  extern char             CommunicationFlt2;            //ͨѶ����2
 
 
 
 /******************************MCU4��������*******************************/
  extern unsigned int VCU4_BYTE_0 ;                //MCU4��һ���ֽڣ����ģʽ
 
  extern char             M_LV_WARN;                   //���Ƿѹ����
  extern char             M_OV_WARN;                   //�����ѹ����
  extern char             M_PRE_CHARG_WARN;            //���Ԥ������
  extern char             M_OTEMP_WARN;                //������¾���
  extern char             M_OVEHICLE_WARN;             //������پ���
  extern char             M_BLOCK_FAULT;               //�����ת����
  extern char             BUS_OCURRENT_FAULT;          //ĸ�߹�������
  extern char             MCU_OTEMP_FAULT;             //MCU���¾���
  
  extern unsigned int VCU4_BYTE_1  ;                    // MCU4�ڶ����ֽڣ����ģʽ
  
  extern char             IGBT_OVTEMP_WARN ;                // IGBT���¾���
  extern char             LV_BETT_LV_FLT ;                  // ��ѹ��ص�ѹ�͹���
  extern char             BUS_CURRENT_SENSOR_Flt ;          // ĸ�ߵ�����������·����
  extern char             BUS_CURRENT_SENSOR_OPEN ;         // ĸ�ߵ�����������·    
  extern char             BUS_CURRENT_SENSOR_OVRANGE_Flt ;  // ĸ�ߵ����������ź�ƫ��ֵ����������Χ
  extern char             BUS_VOLTAGE_SENSOR_OPEN_Flt ;     // ĸ�ߵ�ѹ��������·


#endif

