#if defined(TARGET_KLXX) || defined(TARGET_K20D50M)

#include "FastPWM.h"

void FastPWM::initFastPWM( void ) {
    bits = 16;
}

void FastPWM::pulsewidth_ticks( uint32_t ticks ) {
    *(_pwm.CnV) = ticks;
}

void FastPWM::period_ticks( uint32_t ticks ) {
    *(_pwm.MOD) = ticks - 1;
}

uint32_t FastPWM::getPeriod( void ) {
    return *(_pwm.MOD) + 1;
}

uint32_t FastPWM::setPrescaler(uint32_t reqScale) {
        
    //Yes this is ugly, yes I should feel bad about it
    volatile uint32_t *TPM_SC = _pwm.MOD - 2;
    
    const char prescalers[] = {1, 2, 4, 8, 16, 32, 64, 128};
    
    //If prescaler is 0, return current one
    if (reqScale == 0)
        return (prescalers[(*TPM_SC) & 0x07]);
    
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
    char clockbits = *TPM_SC & (3<<3);
    
    //For some reason clearing them takes some effort
    while ((*TPM_SC & 0x1F) != 0)
        *TPM_SC &= ~0x1F;
        
    
    *TPM_SC = bin + clockbits;
    
    return retval;   
}
#endif