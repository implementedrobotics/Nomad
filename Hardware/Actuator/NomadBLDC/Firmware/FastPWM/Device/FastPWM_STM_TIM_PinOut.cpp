#include "mbed.h"

#ifdef TARGET_NUCLEO_F303RE
__IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin) {
    switch (pin) {
         // Channels 1
        case PC_0: case PB_8: case PB_9: case PA_6: case PA_8: case PB_4: case PB_5: case PA_2: case PC_6: case PA_12: case PB_14: case PB_15:
        // Channels 1N
        case PA_1: case PA_5: case PB_6: case PB_3: case PA_13: case PB_7: case PC_13:
            return &pwm->CCR1;
            
        // Channels 2
        case PC_1: case PA_7: case PC_7: case PA_9: case PA_3: case PA_14:
        // Channels 2N
        case PB_0:
            return &pwm->CCR2;
            
        // Channels 3
        case PA_10: case PC_2: case PC_8:
        // Channels 3N
        case PB_1:
            return &pwm->CCR3;
 
        // Channels 4
        case PC_3: case PC_9: case PA_11:
        // Channels 4N
        
            return &pwm->CCR4;
    }
    return NULL;
}
#endif

#ifdef TARGET_NUCLEO_F030R8
__IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin) {
    switch (pin) {
        // Channels 1
        case PA_4: case PA_6: case PB_1: case PB_4: case PB_8: case PB_9: case PB_14: case PC_6: case PB_6: case PB_7:
            return &pwm->CCR1;
            
        // Channels 2
        case PA_7: case PB_5: case PC_7:
            return &pwm->CCR2;
            
        // Channels 3
        case PB_0: case PC_8:
            return &pwm->CCR3;
            
        // Channels 4
        case PC_9:
            return &pwm->CCR4;
    }        
    return NULL;
}
#endif

#if defined TARGET_NUCLEO_F401RE || defined TARGET_NUCLEO_F411RE || defined TARGET_NUCLEO_F446RE
__IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin) {
    switch (pin) {
        // Channels 1 : PWMx/1
        case PA_0: case PA_5: case PA_6: case PA_8: case PA_15: case PB_4: case PB_6: case PC_6: case PA_7: case PB_13:
            return &pwm->CCR1;
        
        // Channels 2 : PWMx/2
        case PA_1: case PA_9: case PB_3: case PB_5: case PB_7: case PC_7: case PB_0: case PB_14:
            return &pwm->CCR2;
            
        // Channels 3 : PWMx/3
        case PA_2: case PA_10: case PB_8: case PB_10: case PC_8: case PB_1: case PB_15:
            return &pwm->CCR3;
 
        // Channels 4 : PWMx/4
        case PA_3: case PA_11: case PB_9: case PC_9: 
            return &pwm->CCR4;
    }
    return NULL;
}
#endif

#if defined TARGET_NUCLEO_F103RB
__IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin) {
    switch (pin) {
        // Channels 1 : PWMx/1
        case PA_6: case PA_8: case PA_15: case PB_4: case PC_6: case PB_13:
            return &pwm->CCR1;
        
        // Channels 2 : PWMx/2
        case PA_1: case PA_7: case PA_9: case PB_3: case PB_5: case PC_7: case PB_14:
            return &pwm->CCR2;
            
        // Channels 3 : PWMx/3
        case PA_2: case PA_10: case PB_0: case PB_10: case PC_8: case PB_15:
            return &pwm->CCR3;
 
        // Channels 4 : PWMx/4
        case PA_3: case PA_11: case PB_1: case PB_11: case PC_9: 
            return &pwm->CCR4;
    }
    return NULL;
}
#endif

#ifdef TARGET_NUCLEO_F334R8
__IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin) {
    switch (pin) {
         // Channels 1
        case PA_2: case PA_6: case PA_7: case PA_8: case PA_12: case PB_4: case PB_5: case PB_8: case PB_9: case PB_14: case PC_0: case PC_6:
        case PA_1: case PA_13: case PB_6: case PB_13: case PC_13:
            return &pwm->CCR1;
 
        // Channels 2
        case PA_3: case PA_4: case PA_9: case PB_15: case PC_1: case PC_7:
            return &pwm->CCR2;
            
        // Channels 3
        case PA_10: case PB_0: case PC_2: case PC_8:
        case PF_0:
            return &pwm->CCR3;
 
        // Channels 4
        case PA_11: case PB_1: case PB_7: case PC_3: case PC_9:
            return &pwm->CCR4;
    }
    return NULL;
}
#endif

#if defined TARGET_NUCLEO_F072RB
__IO uint32_t* getChannel(TIM_TypeDef* pwm, PinName pin) {
    switch (pin) {
        // Channels 1 : PWMx/1
        case PA_2: case PA_6: case PA_4: case PA_7: case PA_8: case PB_1: case PB_4: case PB_8: case PB_9: case PB_14: case PC_6:
        // Channels 1N : PWMx/1N
        case PA_1: case PB_6: case PB_7: case PB_13:
            return &pwm->CCR1;
        
        // Channels 2 : PWMx/2
        case PA_3: case PA_9: case PB_5: case PC_7: case PB_15:
            return &pwm->CCR2;
            
        // Channels 3 : PWMx/3
        case PA_10: case PB_0: case PC_8: 
            return &pwm->CCR3;
 
        // Channels 4 : PWMx/4
        case PA_11: case PC_9: 
            return &pwm->CCR4;
    }
    return NULL;
}
#endif