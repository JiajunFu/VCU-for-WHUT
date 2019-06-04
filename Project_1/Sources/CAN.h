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
/* struct can_msg                           //发送报文的结构体
{
    unsigned long id;
     char RTR;
    unsigned  char data[8];                //char  data[8];
    unsigned  char data_lenth;                  //报文字节长度
    unsigned  char prty;                   //优先级
};
  */
 
 /******************************VCU1变量定义*******************************/
 extern unsigned char TMEnable ;                           //电机使能 1: 电机使能 ， 0:电机非使能

 extern unsigned char TMFaultReset;                       //电机故障重置请求   1:请求MCU重置故障 ， 0: 停止请求MCU重置故障

 
 /******************************MCU1变量定义*******************************/
  extern unsigned int  TMInvCurrent; //电机母线电流
  extern unsigned int  TMInvVoltage; //电机母线电压
  extern unsigned int  TMInvTemp  ;  //电机逆变器温度
  extern unsigned int  TMTemp;       //电机温度
  extern unsigned int  TM_Mode ;     //MCU1第七个字节，电机模式
  
  extern char              TMReady;             // 电机就绪
  extern char              TMContModeActual;    // 电机当前控制模式
  extern char              MCU_SelfCheckPassed; // MCU自检通过
  extern char              TMInvCurrent_valid;  // 电机母线电流值有效性
  extern char              TMInvVoltage_valid;  // 电机母线电压值有效性
  extern char              TMInvTemp_valid;     // 电机逆变器温度值有效性
  extern char              TMTemp_valid;        // 电机温度值有效性
   
 /******************************MCU2变量定义*******************************/
  extern unsigned int TMTq;                //电机当前转矩
  extern unsigned int TMAllowPosTqMax;     //电机允许最大正向转矩值
  extern unsigned int TMAllowNegTqMax;     //电机允许最大反向转矩值
  extern unsigned int VCU2_BYTE_6;         //MCU2第七个字节，电机模式
 
  extern char             TMTq_valid;              // 电机转矩有效性
  extern char             TMAllowNegTqMin_valid;   // 电机允许最大驱动转矩值有效性
  extern char             TMAllowPosTqMax_valid;   // 电机允许最大发电转矩值有效性
 
 /******************************MCU3变量定义*******************************/
 
  extern unsigned int TMSpd;                 //电机当前转速
  extern unsigned int TMSpd_valid;           //电机当前转速值有效性
  extern unsigned int VCU3_BYTE_3 ;          //MCU3第四个字节，电机模式
  
  extern char             TMOverSpeedFlt;        // 超速故障
  extern char             TMUnderVoltFlt;        // 欠压故障
  extern char             TMOverVoltFlt;         // 过压故障
  extern char             TMOverLoadFlt ;        // 过载故障
  extern char             TMResolverFlt ;        // 旋变故障
  extern char             MCUCANCommFlt ;        // CAN通信故障
  extern char             TMInternalFlt;         // 内部故障
  extern char             TqMonitorFlt  ;        // 扭矩监控故障
  
  extern unsigned int VCU3_BYTE_4 ;          //MCU3第五个字节，电机模式
               
  extern char             TMIGBTFlt1 ;              // IGBT1故障
  extern char             TMIGBTFlt2 ;              // IGBT2故障
  extern char             TMIGBTFlt3 ;              // IGBT3故障
  extern char             TMHWOverCurrFlt1 ;        // 硬件过流故障1
  extern char             TMHWOverCurrFlt2 ;        // 硬件过流故障2
  extern char             TMPhaseACurrSensorFlt ;   // A相电流传感器故障
  extern char             TMPhaseBCurrSensorFlt ;   // B相电流传感器故障
  extern char             DischargeActFlt ;         // 主动放电故障
  
  extern unsigned int VCU3_BYTE_5 ;                 //MCU3第六个字节，电机模式
  
  extern char             TMIGBTTempSensorOCFlt;        //IGBT温度传感器开路
  extern char             MCUTempSensorOCFlt;           //控制器温度传感器开路
  extern char             TMTempSensorOCFlt;            //电机温度传感器开路
  extern char             TMIGBTTempSensorSCFlt;        //IGBT温度传感器短路
  extern char             MCUTempSensorSCFlt;           //控制器温度传感器短路
  extern char             TMTempSensorSCFlt;            //电机温度传感器短路
  extern char             TMRotorPosCheckFlt;           //电机转子位置检查异常
 
  extern unsigned int VCU3_BYTE_6;                 //MCU3第七个字节，电机模式
  
  extern char             TMIGBTOverTempFlt;           //IGBT温度过温
  extern char             MCUOverTempFlt;              //控制器温度过温
  extern char             TMOverTempFlt;               //电机温度过温
  extern char             CommunicationFltl;            //通讯故障1
  extern char             CommunicationFlt2;            //通讯故障2
 
 
 
 /******************************MCU4变量定义*******************************/
  extern unsigned int VCU4_BYTE_0 ;                //MCU4第一个字节，电机模式
 
  extern char             M_LV_WARN;                   //电机欠压警告
  extern char             M_OV_WARN;                   //电机过压警告
  extern char             M_PRE_CHARG_WARN;            //电机预充电故障
  extern char             M_OTEMP_WARN;                //电机过温警告
  extern char             M_OVEHICLE_WARN;             //电机超速警告
  extern char             M_BLOCK_FAULT;               //电机堵转故障
  extern char             BUS_OCURRENT_FAULT;          //母线过流故障
  extern char             MCU_OTEMP_FAULT;             //MCU过温警告
  
  extern unsigned int VCU4_BYTE_1  ;                    // MCU4第二个字节，电机模式
  
  extern char             IGBT_OVTEMP_WARN ;                // IGBT过温警告
  extern char             LV_BETT_LV_FLT ;                  // 低压电池电压低故障
  extern char             BUS_CURRENT_SENSOR_Flt ;          // 母线电流传感器短路故障
  extern char             BUS_CURRENT_SENSOR_OPEN ;         // 母线电流传感器开路    
  extern char             BUS_CURRENT_SENSOR_OVRANGE_Flt ;  // 母线电流传感器信号偏移值超过正常范围
  extern char             BUS_VOLTAGE_SENSOR_OPEN_Flt ;     // 母线电压传感器开路


#endif

