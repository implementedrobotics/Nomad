#ifdef TARGET_LPC1768

#include "FastPWM.h"

void FastPWM::initFastPWM( void ) {
    //Set clock source
    LPC_SC->PCLKSEL0|=1<<12;
    bits = 32;
}

void FastPWM::pulsewidth_ticks( uint32_t ticks ) {
    *(_pwm.MR) = ticks;
    LPC_PWM1->LER |= 1 << _pwm.pwm;
}

void FastPWM::period_ticks( uint32_t ticks ) {
    LPC_PWM1->MR0 = ticks;
    LPC_PWM1->LER |= 1 << 0;
}

uint32_t FastPWM::getPeriod( void ) {
    return LPC_PWM1->MR0;
}

//Maybe implemented later, but needing to change the prescaler for a 32-bit
//timer used in PWM mode is kinda unlikely.
//If you really need to do it, rejoice, you can make it run so slow a period is over 40,000 year
uint32_t FastPWM::setPrescaler(uint32_t reqScale) {
    //Disable dynamic prescaling
    dynamicPrescaler = false;
    
    return 1;
}
#endif