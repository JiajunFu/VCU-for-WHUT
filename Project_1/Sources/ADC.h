#ifndef  __ADC_XS128__
#define  __ADC_XS128__

extern unsigned char AD_value[8];    //����Ĵ���
extern void ADC_Init(void);          //��ʼ��--��ͨ��������ת�� 
extern void AD_Get_value(void);      //��AD�������Ĵ���

extern unsigned int ACC_1;      //���Ŵ�����1
extern unsigned int ACC_2;      //���Ŵ�����2
extern unsigned int ACC;        //���Ŵ�����1,2֮��
extern unsigned int Torque;    //ת�أ�����VCU���ģ��Ե������ת�ؿ���


#endif

