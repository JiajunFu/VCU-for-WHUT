#ifndef  __ADC_XS128__
#define  __ADC_XS128__
#include "lib.h"

extern unsigned char AD_value[8];    //����Ĵ���
void ADC_Init(void);          //��ʼ��--��ͨ��������ת�� 
void AD_Get_value(void);      //��AD�������Ĵ���
uint16 ADCValue(uint8 channel);
extern unsigned int ACC_1;      //���Ŵ�����1
extern unsigned int ACC_2;      //���Ŵ�����2
extern unsigned int ACC;        //���Ŵ�����1,2֮��
extern unsigned int Torque;    //ת�أ�����VCU���ģ��Ե������ת�ؿ���


#endif

