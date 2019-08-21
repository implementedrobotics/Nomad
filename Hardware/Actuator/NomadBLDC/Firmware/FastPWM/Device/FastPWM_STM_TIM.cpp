//This should (hopefully) work on all STM targets which use TIM timers for PWM

#ifdef TARGET_STM

#include "FastPWM.h"

typedef __IO uint32_t* CHANNEL_P_T;

#define PWM_CHANNEL     (**(CHANNEL_P_T*)fast_obj)
#define PWM_TIMER       ((TIM_TypeDef*)_pwm.pwm)

extern CHANNEL_P_T getChannel(TIM_TypeDef* pwm, PinName pin);

void FastPWM::initFastPWM( void ) {
    fast_obj = new (CHANNEL_P_T);
    *(CHANNEL_P_T*)fast_obj = getChannel(PWM_TIMER, _pwm.pin);
    
    //Enable PWM period syncing for glitch free result
    PWM_TIMER->CR1 |= TIM_CR1_ARPE;
    
    bits = 16;
}

void FastPWM::pulsewidth_ticks( uint32_t ticks ) {
    PWM_CHANNEL = ticks;    
}

void FastPWM::period_ticks( uint32_t ticks ) {
    PWM_TIMER->ARR = ticks - 1;
}

uint32_t FastPWM::getPeriod( void ) {
    return PWM_TIMER->ARR + 1;
}

uint32_t FastPWM::setPrescaler(uint32_t reqScale) {
    if (reqScale == 0)
        //Return prescaler
        return PWM_TIMER->PSC + 1;
    if (reqScale > (uint32_t)(1<<16))
        reqScale = 1<<16;
    //Else set prescaler, we have to substract one from reqScale since a 0 in PCVAL is prescaler of 1
    PWM_TIMER->PSC = reqScale - 1;

    return reqScale;
}

#endif