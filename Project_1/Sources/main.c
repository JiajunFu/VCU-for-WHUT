#include <hidef.h>      /* common defines and macros */
#include "derivative.h"      /* derivative-specific definitions */
#include "IO.h"
#include "PLL.h"
#include "ECT.h"
#include "ADC.h"
#include "CAN.h"
#include "PIT.h"
#define PedMax 0.1           
#define PedUpLimit 5  		//踏板行程电压范围最大
unsigned int ACC_1;      //油门传感器1
unsigned int ACC_2;      //油门传感器2
unsigned int Torque;     //转矩
void Drive(void);
/***开关量输入***/
unsigned char on_fire ;   //ON火常闭。
unsigned char brake;      //制动
unsigned char start;      //启动。点动或常闭

/***开关量输出***/
unsigned char Pre_charge; //电机预充电
unsigned char charge;     //控制主继电器
unsigned char buzz;       //鸣笛   
           
 /******************************VCU1变量定义*******************************/
  unsigned char TMEnable;    //电机使能 1: 电机使能 ， 0:电机非使能

  unsigned char TMFaultReset; //电机故障重置请求   1:请求MCU重置故障 ， 0: 停止请求MCU重置故障
 
 
 /******************************MCU1变量定义*******************************/
   
   
   unsigned int  TMInvCurrent; //电机母线电流
   unsigned int  TMInvVoltage; //电机母线电压
   unsigned int  TMInvTemp  ;  //电机逆变器温度
   unsigned int  TMTemp;       //电机温度
   unsigned int  TM_Mode ;     //MCU1第七个字节，电机模式
  
   char              TMReady;             // 电机就绪
   char              TMContModeActual;    // 电机当前控制模式
   char              MCU_SelfCheckPassed; // MCU自检通过
   char              TMInvCurrent_valid;  // 电机母线电流值有效性
   char              TMInvVoltage_valid;  // 电机母线电压值有效性
   char              TMInvTemp_valid;     // 电机逆变器温度值有效性
   char              TMTemp_valid;        // 电机温度值有效性
   
 /******************************MCU2变量定义*******************************/
   unsigned int TMTq;                //电机当前转矩
   unsigned int TMAllowPosTqMax;     //电机允许最大正向转矩值
   unsigned int TMAllowNegTqMax;     //电机允许最大反向转矩值
   unsigned int VCU2_BYTE_6;         //MCU2第七个字节，电机模式
 
   char             TMTq_valid;              // 电机转矩有效性
   char             TMAllowNegTqMin_valid;   // 电机允许最大驱动转矩值有效性
   char             TMAllowPosTqMax_valid;   // 电机允许最大发电转矩值有效性
 
 /******************************MCU3变量定义*******************************/
 
   unsigned int TMSpd;                 //电机当前转速
   unsigned int TMSpd_valid;           //电机当前转速值有效性
   unsigned int VCU3_BYTE_3 ;          //MCU3第四个字节，电机模式
  
   char             TMOverSpeedFlt;        // 超速故障
   char             TMUnderVoltFlt;        // 欠压故障
   char             TMOverVoltFlt;         // 过压故障
   char             TMOverLoadFlt ;        // 过载故障
   char             TMResolverFlt ;        // 旋变故障
   char             MCUCANCommFlt ;        // CAN通信故障
   char             TMInternalFlt;         // 内部故障
   char             TqMonitorFlt  ;        // 扭矩监控故障
  
   unsigned int VCU3_BYTE_4 ;          //MCU3第五个字节，电机模式
               
   char             TMIGBTFlt1 ;              // IGBT1故障
   char             TMIGBTFlt2 ;              // IGBT2故障
   char             TMIGBTFlt3 ;              // IGBT3故障
   char             TMHWOverCurrFlt1 ;        // 硬件过流故障1
   char             TMHWOverCurrFlt2 ;        // 硬件过流故障2
   char             TMPhaseACurrSensorFlt ;   // A相电流传感器故障
   char             TMPhaseBCurrSensorFlt ;   // B相电流传感器故障
   char             DischargeActFlt ;         // 主动放电故障
  
   unsigned int VCU3_BYTE_5 ;                 //MCU3第六个字节，电机模式
  
   char             TMIGBTTempSensorOCFlt;        //IGBT温度传感器开路
   char             MCUTempSensorOCFlt;           //控制器温度传感器开路
   char             TMTempSensorOCFlt;            //电机温度传感器开路
   char             TMIGBTTempSensorSCFlt;        //IGBT温度传感器短路
   char             MCUTempSensorSCFlt;           //控制器温度传感器短路
   char             TMTempSensorSCFlt;            //电机温度传感器短路
   char             TMRotorPosCheckFlt;           //电机转子位置检查异常
 
   unsigned int VCU3_BYTE_6;                 //MCU3第七个字节，电机模式
  
   char             TMIGBTOverTempFlt;           //IGBT温度过温
   char             MCUOverTempFlt;              //控制器温度过温
   char             TMOverTempFlt;               //电机温度过温
   char             CommunicationFltl;            //通讯故障1
   char             CommunicationFlt2;            //通讯故障2
 
 /******************************MCU4变量定义*******************************/
   unsigned int VCU4_BYTE_0 ;                //MCU4第一个字节，电机模式
 
   char             M_LV_WARN;                   //电机欠压警告
   char             M_OV_WARN;                   //电机过压警告
   char             M_PRE_CHARG_WARN;            //电机预充电故障
   char             M_OTEMP_WARN;                //电机过温警告
   char             M_OVEHICLE_WARN;             //电机超速警告
   char             M_BLOCK_FAULT;               //电机堵转故障
   char             BUS_OCURRENT_FAULT;          //母线过流故障
   char             MCU_OTEMP_FAULT;             //MCU过温警告
  
   unsigned int VCU4_BYTE_1  ;                    // MCU4第二个字节，电机模式
  
   char             IGBT_OVTEMP_WARN ;                // IGBT过温警告
   char             LV_BETT_LV_FLT ;                  // 低压电池电压低故障
   char             BUS_CURRENT_SENSOR_Flt ;          // 母线电流传感器短路故障
   char             BUS_CURRENT_SENSOR_OPEN ;         // 母线电流传感器开路    
   char             BUS_CURRENT_SENSOR_OVRANGE_Flt ;  // 母线电流传感器信号偏移值超过正常范围
   char             BUS_VOLTAGE_SENSOR_OPEN_Flt ;     // 母线电压传感器开路



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
/*                         主函数                            */
/*************************************************************/
 void main(void)
{  
   unsigned char start_flg=0;
   DisableInterrupts;
    Init_all() ;
 
   EnableInterrupts; 
   on_fire = SIN_1;              //都是外部输入信号,ON火
   start   = SIN_2;              //启动
   brake   = SIN_3;              //制动
  
  
  
   W_1 = charge;            //都是输出信号，控制外部主继电器
   W_2 = buzz;                  //鸣笛 
   W_3 = Pre_charge;            //电机预充电
   
   
   ACC_1= ((unsigned int)(AD_value[1])<<8)|(AD_value[0]);
   ACC_2= ((unsigned int)(AD_value[3])<<8)|(AD_value[2]);  
   
   	if(on_fire)     // on_fire为点动开关 
		{
			   charge=0;                             // 主继电器断开
                Pre_charge=1;                         // 电机预充电，对电机起到保护作用        
                delay1ms( 700) ;                          // 延时0.7秒
                 Pre_charge=0;                             // 关闭预充电    
                 charge=1;           
		}
       
    	while(1)
	{
     	if((brake&start)|start_flg)
			{
				if((brake&start)&&(start_flg==0))  //判断第一次进入待驶状态。制动踏板，on火输入
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
	if((ACC_1-ACC_2)>PedMax)  //传感器故障检测。2个传感器信号之间差值大于一定值，认为有故障
	{
	Torque=0;
	}
	else if(brake|PedWait_flg)  //
	{
	Torque=0;
		if(ACC_1>(0.25*PedUpLimit))//当油门开度踩到25%以下，且踩下制动踏板，必须松油门才能再输出转矩
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
		Trq_re_temp=(unsigned int)(ACC_1*40);//具体大小需要调试。扭矩范围0-200N
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



 
 




