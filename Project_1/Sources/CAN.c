/////////////////////////////////////////////////////////////////////////
//                                                                     //
//注意：CAN的中断要在配置模式之外CAN发送要清TXE                      //
/////////////////////////////////////////////////////////////////////////


#include <hidef.h>      /* common defines and macros */
#include <mc9s12XS128.h>
#include "derivative.h" 
#include "CAN.h"
#include "ADC.h"

unsigned char  j,k;
  
#define ID1                 0x8FF20EF   //VCU发送报文1
#define ID2                 0x8FF22EF   //VCU发送报文2

#define ID3                 0xCFF30F0   //接收MCU报文1
#define ID4                 0xCFF32F0   //接收MCU报文2
#define ID5                 0xCFF34F0   //接收MCU报文3
#define ID6                 0xCFF36F0   //接收MCU报文4
struct can_msg                           //发送报文的结构体
{
    unsigned long id;
    char RTR;
    unsigned  char data[8];                //char  data[8];
    unsigned  char data_lenth;                  //报文字节长度
    unsigned  char prty;                   //优先级
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
/*                        CAN读取数据                        */
/*************************************************************/
void Read_Can(void)
{ if(rmsg.id==0xCFF30F0) 
 {
   TMInvCurrent= (((( unsigned int)(rmsg.data[1]))<<8)|(rmsg.data[0])); //电机母线电流
   TMInvVoltage= (((( unsigned int)(rmsg.data[3]))<<8)|(rmsg.data[2])); //电机母线电压
   TMInvTemp= rmsg.data[4] ;  // 电机逆变器温度
   TMTemp= rmsg.data[5] ;     // 电机温度
   
   TM_Mode= rmsg.data[6] ;
   TMReady= TM_Mode&0x01;             // 电机就绪
   TMContModeActual= ((TM_Mode&0x06)>>1);    // 电机当前控制模式
   MCU_SelfCheckPassed= ((TM_Mode&0x08)>>3); // MCU自检通过
   TMInvCurrent_valid=  ((TM_Mode&0x10)>>4); // 电机母线电流值有效性
   TMInvVoltage_valid=  ((TM_Mode&0x20)>>5); // 电机母线电压值有效性
   TMInvTemp_valid=     ((TM_Mode&0x40)>>6); // 电机逆变器温度值有效性
   TMTemp_valid=        ((TM_Mode&0x80)>>7); // 电机温度值有效性
   
   Life= ((rmsg.data[7])&0x0f) ;              // MCU Message1生命信号
 }
  else if(rmsg.id==0xCFF32F0) 
  {
   TMTq= (((unsigned int)(rmsg.data[1]))<<8)|(rmsg.data[0]);              //电机当前转矩
   TMAllowPosTqMax= (((unsigned int)(rmsg.data[3]))<<8)|(rmsg.data[2]);   //电机允许最大正向转矩值
   TMAllowNegTqMax= (((unsigned int)(rmsg.data[5]))<<8)|(rmsg.data[4]);   //电机允许最大反向转矩值
  
   VCU2_BYTE_6= rmsg.data[6] ;
   TMTq_valid=  ((VCU2_BYTE_6&0x01)>>0);             // 电机转矩有效性
   TMAllowNegTqMin_valid= ((VCU2_BYTE_6&0x02)>>1);   // 电机允许最大驱动转矩值有效性
   TMAllowPosTqMax_valid= ((VCU2_BYTE_6&0x04)>>2);   // 电机允许最大发电转矩值有效性
                            
   Life= ((rmsg.data[7])&0x0f) ;              // MCU Message2生命信号
  }
  
  else if(rmsg.id==0xCFF34F0) 
 {
  TMSpd= (((unsigned int)(rmsg.data[1]))<<8)|(rmsg.data[0]);      //电机当前转速
  TMSpd_valid= ((rmsg.data[2])&0x01) ;                            //电机当前转速值有效性
  
  VCU3_BYTE_3= rmsg.data[3] ;
  TMOverSpeedFlt= VCU3_BYTE_3&0x01;    // 超速故障
  TMUnderVoltFlt= ((VCU3_BYTE_3&0x02)>>1);    // 欠压故障
  TMOverVoltFlt = ((VCU3_BYTE_3&0x04)>>2);    // 过压故障
  TMOverLoadFlt = ((VCU3_BYTE_3&0x08)>>3);    // 过载故障
  TMResolverFlt = ((VCU3_BYTE_3&0x10)>>4);    // 旋变故障
  MCUCANCommFlt = ((VCU3_BYTE_3&0x20)>>5);    // CAN通信故障
  TMInternalFlt = ((VCU3_BYTE_3&0x40)>>6);    // 内部故障
  TqMonitorFlt  = ((VCU3_BYTE_3&0x80)>>7);    // 扭矩监控故障
  
  VCU3_BYTE_4= rmsg.data[4] ;
  TMIGBTFlt1=  VCU3_BYTE_4&0x01;            // IGBT1故障
  TMIGBTFlt2=  ((VCU3_BYTE_4&0x02)>>1);            // IGBT2故障
  TMIGBTFlt3=  ((VCU3_BYTE_4&0x04)>>2);            // IGBT3故障
  TMHWOverCurrFlt1= ((VCU3_BYTE_4&0x08)>>3);       // 硬件过流故障1
  TMHWOverCurrFlt2= ((VCU3_BYTE_4&0x10)>>4);       // 硬件过流故障2
  TMPhaseACurrSensorFlt=  ((VCU3_BYTE_4&0x20)>>5); // A相电流传感器故障
  TMPhaseBCurrSensorFlt=  ((VCU3_BYTE_4&0x40)>>6); // B相电流传感器故障
  DischargeActFlt= ((VCU3_BYTE_4&0x80)>>7);        // 主动放电故障
  
  VCU3_BYTE_5= rmsg.data[5] ;
  TMIGBTTempSensorOCFlt=  VCU3_BYTE_5&0x01;       //IGBT温度传感器开路
  MCUTempSensorOCFlt=  ((VCU3_BYTE_5&0x02)>>1);          //控制器温度传感器开路
  TMTempSensorOCFlt=  ((VCU3_BYTE_5&0x04)>>2);           //电机温度传感器开路
  TMIGBTTempSensorSCFlt= ((VCU3_BYTE_5&0x08)>>3);        //IGBT温度传感器短路
  MCUTempSensorSCFlt= ((VCU3_BYTE_5&0x10)>>4);           //控制器温度传感器短路
  TMTempSensorSCFlt=  ((VCU3_BYTE_5&0x20)>>5);           // 电机温度传感器短路
  TMRotorPosCheckFlt=  ((VCU3_BYTE_5&0x40)>>6);          // 电机转子位置检查异常
 
  VCU3_BYTE_6= rmsg.data[6] ;
  TMIGBTOverTempFlt=  VCU3_BYTE_6&0x01;       //IGBT温度过温
  MCUOverTempFlt=  ((VCU3_BYTE_6&0x02)>>1);          //控制器温度过温
  TMOverTempFlt=  ((VCU3_BYTE_6&0x04)>>2);           //电机温度过温
 
  Life= ((rmsg.data[7])&0x0f) ;               // MCU Message3生命信号
 }
  
  else if(rmsg.id==0xCFF36F0) 
 {
  VCU4_BYTE_0= rmsg.data[0] ;
  M_LV_WARN=  VCU4_BYTE_0&0x01;               //电机欠压警告
  M_OV_WARN=  ((VCU4_BYTE_0&0x02)>>1);               //电机过压警告
  M_PRE_CHARG_WARN=  ((VCU4_BYTE_0&0x04)>>2);        //电机预充电故障
  M_OTEMP_WARN=  ((VCU4_BYTE_0&0x08)>>3);            //电机过温警告
  M_OVEHICLE_WARN=  ((VCU4_BYTE_0&0x10)>>4);         //电机超速警告
  M_BLOCK_FAULT=  ((VCU4_BYTE_0&0x20)>>5);           //电机堵转故障
  BUS_OCURRENT_FAULT=  ((VCU4_BYTE_0&0x40)>>6);      //母线过流故障
  MCU_OTEMP_FAULT=  ((VCU4_BYTE_0&0x80)>>7);         //MCU过温警告
  
  VCU4_BYTE_1= rmsg.data[1] ;
  IGBT_OVTEMP_WARN=  VCU4_BYTE_1&0x01;                //IGBT过温警告
  LV_BETT_LV_FLT=    ((VCU4_BYTE_1&0x02)>>1);                //低压电池电压低故障
  BUS_CURRENT_SENSOR_Flt=  ((VCU4_BYTE_1&0x04)>>2);          //母线电流传感器短路故障
  BUS_CURRENT_SENSOR_OPEN=  ((VCU4_BYTE_1&0x08)>>3);         //母线电流传感器开路    
  BUS_CURRENT_SENSOR_OVRANGE_Flt = ((VCU4_BYTE_1&0x10)>>4);  // 母线电流传感器信号偏移值超过正常范围
  BUS_VOLTAGE_SENSOR_OPEN_Flt=  ((VCU4_BYTE_1&0x20)>>5);     // 母线电压传感器开路
 }
  else 
    ;       
}

/*************************************************************/
/*                        CAN初始化                          */
/*************************************************************/
void CAN_Init(void) 
{
  if(CAN0CTL0_INITRQ==0)       // 查询是否进入初始化状态   
    CAN0CTL0_INITRQ =1;        // 进入初始化状态

  while (CAN0CTL1_INITAK==0);  // 等待进入初始化状态

  CAN0BTR0_SJW = 0;            //设置同步
  CAN0BTR0_BRP = 7;            //设置波特率  
  CAN0BTR1 = 0x1c;             //设置时段1和时段2的Tq个数 ,波特率为250kb/s

  // 关闭滤波器                                  
  CAN0IDMR0 = 0xFF;
  CAN0IDMR1 = 0xFF;
  CAN0IDMR2 = 0xFF;
  CAN0IDMR3 = 0xFF;
  CAN0IDMR4 = 0xFF;
  CAN0IDMR5 = 0xFF;
  CAN0IDMR6 = 0xFF;
  CAN0IDMR7 = 0xFF; 

  CAN0CTL1 = 0xc0;             //使能MSCAN模块,设置为一般运行模式、使用总线时钟源 

  CAN0CTL0 = 0x00;             //返回一般模式运行

  while(CAN0CTL1_INITAK);      //等待回到一般运行模式

  while(CAN0CTL0_SYNCH==0);    //等待总线时钟同步

  CAN0RIER_RXFIE = 0;          //0:禁止接收中断; 1:使能接收中断 ,报文成功接收触发中断
}

/*************************************************************/
/*                       CAN发送                             */
/*************************************************************/
unsigned char MSCAN0SendMsg(struct can_msg tmsg)
{
  unsigned char send_buf,t,sp;
 // unsigned  char data_lenth=8;
  // 检查数据长度
  if(tmsg.data_lenth > 8)
    return(FALSE); 

  // 检查总线时钟
  if(CAN0CTL0_SYNCH==0)
    return(FALSE);

  send_buf = 0;
  do
  {
    // 寻找空闲的缓冲器
    CAN0TBSEL=CAN0TFLG;
    send_buf=CAN0TBSEL;
  }
  
  while(!send_buf); 
  
 
  if(tmsg.RTR)
    // RTR = 阴性
    CAN0TXIDR3 |= 0x01;
    
    // CAN写入数据长度，8字节  
   CAN0TXDLR=tmsg.data_lenth; 
   
    // CAN写入数据
   for(sp = 0; sp < data_lenth; sp++)
     *((&CAN0TXDSR0)+sp) = tmsg.data[sp]; 
  
  
  /***CAN写入ID***/
  CAN0TXIDR0 = (unsigned char)(tmsg.id>>21);
  CAN0TXIDR1 = (((unsigned char)(tmsg.id>>13))&0xe0)|0x18|(((unsigned char)(tmsg.id>>15))&0x07);
  CAN0TXIDR2 = (unsigned char)(tmsg.id>>7);
  CAN0TXIDR3 = ((unsigned char)(tmsg.id<<1));  
   
  // 写入优先级
  //CAN0TXTBPR = msg.prty;
  
  // 清 TXx 标志 (缓冲器准备发送)
  CAN0TFLG = send_buf;
  
  if(CAN0TFLG)
     t=1;
   else
     t=0;
    return(t);       //t作为返回值，作为CAN_send_ready()函数中判断报文是否发送成功的依据
  
} 


/*************************************************************/
/*                       CAN接收                            */
/*************************************************************/
unsigned char MSCAN0GetMsg(struct can_msg rmsg)
{
  unsigned char tp;

  // 检测接收标志
  if(!(CAN0RFLG_RXF))
    return(FALSE);
  
  // 检测 CAN协议报文模式 （一般/扩展） 标识符
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
  
  // 读取数据长度 
  rmsg.data_lenth = CAN0RXDLR;
  
  // 读取数据
  for(tp = 0; tp < rmsg.data_lenth; tp++)
    rmsg.data[tp] = *((&CAN0RXDSR0)+tp);

  // 清 RXF 标志位 (缓冲器准备接收)
  CAN0RFLG = 0x01;


  Read_Can();                  //读取CAN数据，如果CAN中断中不能够正常接收报文，就在MSCAN0GetMsg()
                                 //中接收报文
}

/*************************************************************/
/*                       CAN发送数据准备                     */
/*************************************************************/     
void CAN_send_ready( void)

{  //电机参数     25/55kW 	70/200Nm	3000/9500	320(额定电压)	265-370(电压范围)

   //对VCU1，VCU2进行赋值 ,在中断程序里发送VCU1，VCU2
    /************** 整车控制器发送报文VCU1****************/
  
 
    VCU_1_senddata[0]=(0x65)|(TMEnable<<2) |(TMFaultReset<<5);            //0x65二进制为01100101
    //证明定义外部变量ACC后，可以将main函数中的ACC值赋给CAN.c函数中的VCU_1_senddata[0]   
    // 01000101即为0x45
  /*
  
     Bit0	电机控制模式请求（TMContMode）	0 : 待机模式（STAND BY MODE）
     Bit1                                 1 : 转矩控制模式（TQ CTL MODE）
                                          2 : 转速控制模式（SPD CTL MODE）
                                          3 : 跛行（LimpHome）
   		
     Bit2	电机使能（TMEnable）          	1: 电机使能
                                          0:电机非使能
     Bit3	电机主动放电请求（TMActiveDischarge）  	1:主动放电使能
                                                  0:主动放电非使能
     Bit5	电机故障重置请求（TMFaultReset）      	1:请求MCU重置故障       有点没理解!!!
                                                  0:停止请求MCU重置故障
                                          
     Bit6	电机转矩需求有效性（TMTqReq_valid）	  1 : 信号有效
                                                0 : 信号无效
     Bit7	电机转速需求有效性（TMSpdReq_valid）	1 : 信号有效
                                                0 : 信号无效
   */
 
  VCU_1_senddata[1]=(unsigned char)(((Torque+3000)*10)&0x00ff);       // 默认为转矩低8位, 高低字节在协议中没有定义，还不能确定 
  VCU_1_senddata[2]=(unsigned char)(((Torque+3000)*10)>>8);           // 默认为转矩高8位, 来自于油门信号，Torque的值
  //BYTE 1,BYTE 2 ,电机转矩请求（TMTqReq）  from Torque信号
  
  /*信号物理值与逻辑值的转换公式为：信号物理值 = 比例因子×信号逻辑值+ 偏移量
 	即，如果转速信号的比例因子为0.25，偏移量为0，则有下述转换关系：
 	2000rpm = 8000（1F40h） * 0.25 + 0 
  */
  
  VCU_1_senddata[3]=0;
  VCU_1_senddata[4]=0;       //BYTE3	BYTE4   电机转速请求（TMSpdReq）
  	                         //偏移量-3000，比例因子1
                             //由于采用转矩输出模式，就不必控制转速
                             
  VCU_1_senddata[5]=0x00;                        
  VCU_1_senddata[6]=0x00   ; 
  //BYTE5	 BYTE6 ，这两个字节无效
 
  VCU_1_senddata[7]=Life;       
  //BYTE7	VCU Message1 生命信号
 
  
  /************** 整车控制器发送报文VCU2****************/
  VCU_2_senddata[0]=0       ;
  VCU_2_senddata[1]=0       ;               //需要仔细考虑！！！
  //BYTE0	 BYTE1  电机转矩变化率限值（TMTqSlewRate）
   
  VCU_2_senddata[2]=(unsigned char)(((TMAllowPosTqMax+3000)*10)&0x00ff);     
  VCU_2_senddata[3]=(unsigned char)(((TMAllowPosTqMax+3000)*10)>>8);                   
  //BYTE2	BYTE3  正向转矩限值（PosTqLimit） 用于限制正向转矩，正值
    
  VCU_2_senddata[4]=(unsigned char)(((0+3000)*10)&0x00ff);     
  VCU_2_senddata[5]=(unsigned char)(((0+3000)*10)>>8);                  
  //BYTE4 BYTE5	反向转矩限值（NegTqLimit） 用于限制负向转矩，负值
  //赛车不能够正向或者反向行驶，只能一个方向，这里反向转矩设为0
  
  VCU_2_senddata[6]=0x00       ; //BYTE6无效字节
  
  VCU_2_senddata[7]=Life       ;

  //BYTE7	Bit0  	Bit1   	Bit2  Bit3   VCU Message2 生命信号
  
  
 
   if(send_ctr==1)               //发送VCU1
      { msg.id = ID1;  
        msg.data_lenth=8;
         for(j=0;j<data_lenth;j++)
        {
          msg.data[j] = VCU_1_senddata[j];
        }
   
        if(!(MSCAN0SendMsg(msg)))   //发送过程出现错误
           ;
        
         else
          ;
      } 
      
     
    else if (send_ctr==0)       //发送VCU2
      { msg.id = ID2; 
        msg.data_lenth=8;
    
        for(k=0;k<data_lenth;k++)
        {
            msg.data[k] = VCU_2_senddata[k];
         }    
     
       if(!MSCAN0SendMsg(msg))     //发送过程出现错误       
          ;
      else
          ;
      }
     else
      ; 
          
}

/*************************************************************/
/*                       CAN中断处理                         */
/*************************************************************/

#pragma CODE_SEG NON_BANKED
interrupt VectorNumber_Vcan0rx void CANReceive_ISR (void)                 // CAN接收中断
{ 
  DisableInterrupts;
  
  
  Read_Can();            //读取CAN数据
  CAN0RFLG_RXF=0;        //接收缓冲寄存器清零,释放接收缓冲器
  
  
  EnableInterrupts; 
}

/*interrupt VectorNumber_Vcan0err void CANErr_ISR (void)                  // CAN错误帧处理中断
{
  DisableInterrupts;
  
  CAN_Init();
  EnableInterrupts;
 
}
*/
#pragma CODE_SEG DEFAULT
