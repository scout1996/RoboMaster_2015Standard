#ifndef BUZZER_H
#define BUZZER_H

#include "main.h"

extern void buzzer_init(uint16_t arr, uint16_t psc);  //·äÃùÆ÷³õÊ¼»¯

extern void buzzer_on(uint16_t psc, uint16_t pwm);  //·äÃùÆ÷¿ª
extern void buzzer_off(void);   //·äÃùÆ÷¹Ø

#endif
