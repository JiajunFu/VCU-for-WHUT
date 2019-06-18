#ifndef IO_H
#define IO_H

extern  void IO_Init(void);

#define W     PORTA
#define W_DR  DDRA
#define W_1   DDRA_DDRA0
#define W_2   DDRA_DDRA1
#define W_3   DDRA_DDRA2
#define W_4   DDRA_DDRA3
#define W_5   DDRA_DDRA4
#define W_6   DDRA_DDRA5
 // A口为开关量输出 ,控制继电器开断


#define SIN      PORTB
#define SIN_DR   DDRB
#define SIN_1    DDRB_DDRB0
#define SIN_2    DDRB_DDRB1
#define SIN_3    DDRB_DDRB2
#define SIN_4    DDRB_DDRB3
#define SIN_5    DDRB_DDRB4
#define SIN_6    DDRB_DDRB5
 // B口为开关量输入


void    gpio_set   (char IO,           uint8 data);    //设置引脚

uint8   gpio_get   (char IO);                          //??????
#endif
#endif
