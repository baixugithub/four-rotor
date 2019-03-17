#ifndef __Ultrasonic_H_
#define __Ultrasonic_H_

#include "include.h"


extern float US100_Alt;
extern float US100_Alt_V;

extern float g_HightPwm;

void Ultrasonic_Config(void);
void Ultrasonic_Pulsing(void);

void Filter_Hight(u16 set_hight);
void Hight_PwmOut(void);
#endif // __Ultrasonic_H__
