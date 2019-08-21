#ifndef HW_SETUP_H
#define HW_SETUP_H

#include "mbed.h"
#include "structs.h"

void Init_PWM(GPIOStruct *gpio);
void Init_ADC(void);
void Init_DAC(void);
void Init_All_HW(GPIOStruct *gpio);

#endif