#include"MC9S12XS128.h"




/*************************************************************/
/*                      ��ʼ�����໷                         */
/*************************************************************/
void PLL_Init(void) 
{
    CLKSEL &= 0x7f;       //set OSCCLK as sysclk  &�룬��������λ�ܵ�Ӱ��
    PLLCTL &= 0x8F;       //Disable PLL circuit
    CRGINT &= 0xDF;
    
    #if(BUS_CLOCK == 40000000) 
      SYNR = 0x44;
    #elif(BUS_CLOCK == 32000000)
      SYNR = 0x43;     
    #elif(BUS_CLOCK == 24000000)
      SYNR = 0x42;
    #endif    

    REFDV = 0x81;         //PLLCLK=2��OSCCLK��(SYNR+1)/(REFDV+1)��64MHz ,fbus=32M    //����ʱ��32MHz
    PLLCTL =PLLCTL|0x70;  //Enable PLL circuit
    asm NOP;
    asm NOP;
    while(!(CRGFLG&0x08)); //PLLCLK is Locked already      
    CLKSEL |= 0x80;        //set PLLCLK as sysclk
}
