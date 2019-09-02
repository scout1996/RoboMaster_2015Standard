#ifndef __LED_H
#define __LED_H
#include "sys.h"

#define LED0 PBout(0)
#define LED1 PBout(1)

#define LED0_ON PBout(0)=0;
#define LED1_ON PBout(1)=0;
#define LED0_OFF PBout(0)=1;
#define LED1_OFF PBout(1)=1;

void LED_Init(void);//≥ı ºªØ


#endif
