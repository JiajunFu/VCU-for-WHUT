#ifndef  __ADC_XS128__
#define  __ADC_XS128__

extern unsigned char AD_value[8];    //结果寄存器
extern void ADC_Init(void);          //初始化--多通道，单次转换 
extern void AD_Get_value(void);      //将AD结果存入寄存器

extern unsigned int ACC_1;      //油门传感器1
extern unsigned int ACC_2;      //油门传感器2
extern unsigned int ACC;        //油门传感器1,2之和
extern unsigned int Torque;    //转矩，赋给VCU报文，对电机进行转矩控制


#endif

