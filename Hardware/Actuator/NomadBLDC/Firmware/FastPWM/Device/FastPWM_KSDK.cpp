#if defined(TARGET_KPSDK_MCUS)

#include "FastPWM.h"
#include "fsl_clock_manager.h"


#define PWM_CNV              (*(((fastpwm_struct*)fast_obj)->CnV))  
#define PWM_MOD              (*(((fastpwm_struct*)fast_obj)->MOD))  
#define PWM_SC               (*(((fastpwm_struct*)fast_obj)->SC))  

typedef struct  {
    __IO uint32_t *CnV;
    __IO uint32_t *MOD;
    __IO uint32_t *SC;
} fastpwm_struct;

static uint32_t pwm_prescaler;

void FastPWM::initFastPWM( void ) {
    fast_obj = new fastpwm_struct;
    bits = 16;
    
    uint32_t pwm_base_clock;
    CLOCK_SYS_GetFreq(kBusClock, &pwm_base_clock);
    pwm_prescaler = SystemCoreClock / pwm_base_clock;

    uint32_t ftms[] = FTM_BASE_ADDRS;
    unsigned int ch_n = (_pwm.pwm_name & 0xFF);
    FTM_Type *ftm = (FTM_Type *)ftms[_pwm.pwm_name >> TPM_SHIFT];
    
    ((fastpwm_struct*)fast_obj)->CnV = &ftm->CONTROLS[ch_n].CnV;
    ((fastpwm_struct*)fast_obj)->MOD = &ftm->MOD;
    ((fastpwm_struct*)fast_obj)->SC = &ftm->SC;
}

void FastPWM::pulsewidth_ticks( uint32_t ticks ) {
    PWM_CNV = ticks;
}

void FastPWM::period_ticks( uint32_t ticks ) {
    PWM_MOD = ticks - 1;
}

uint32_t FastPWM::getPeriod( void ) {
    return PWM_MOD + 1;
}

uint32_t FastPWM::setPrescaler(uint32_t reqScale) {
 
    uint32_t prescalers[] = {1, 2, 4, 8, 16, 32, 64, 128};
    
    for (int i = 0; i<8; i++)
         prescalers[i] = prescalers[i] * pwm_prescaler;
    
    //If prescaler is 0, return current one
    if (reqScale == 0)
        return (prescalers[(PWM_SC) & 0x07]);
    
    uint32_t retval = 0;
    char bin;
    
    for (bin = 0; bin<8; bin++) {
        retval = prescalers[bin];
        if (retval >= reqScale)
            break;
    }
    if (bin == 8)
        bin = 7;
    
    //Clear lower 5 bits, write new value:
    char clockbits = PWM_SC & (3<<3);
    
    //For some reason clearing them takes some effort
    while ((PWM_SC & 0x1F) != 0)
        PWM_SC &= ~0x1F;
        
    
    PWM_SC = bin + clockbits;
    
    return retval;   
}
#endif