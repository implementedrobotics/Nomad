#if defined(TARGET_LPC11UXX) || defined(TARGET_LPC11XX_11CXX)

#include "FastPWM.h"

#define PWM_MR              (*(((fastpwm_struct*)fast_obj)->MR))   
#define PWM_TIMER           (((fastpwm_struct*)fast_obj)->timer)      

typedef struct {
    uint8_t timer;
    uint8_t mr;
} timer_mr;
 
#ifdef TARGET_LPC11UXX
typedef struct  {
    __IO uint32_t *MR;
    LPC_CTxxBx_Type *timer;
} fastpwm_struct;

static timer_mr pwm_timer_map[11] = {
    {0, 0}, {0, 1}, {0, 2},
    {1, 0}, {1, 1},
    {2, 0}, {2, 1}, {2, 2},
    {3, 0}, {3, 1}, {3, 2},
};

static LPC_CTxxBx_Type *Timers[4] = {
    LPC_CT16B0, LPC_CT16B1,
    LPC_CT32B0, LPC_CT32B1
};
#else           //LPC11XX
typedef struct  {
    __IO uint32_t *MR;
    LPC_TMR_TypeDef *timer;
} fastpwm_struct;

static timer_mr pwm_timer_map[5] = {
    {0, 0}, /* CT16B0, MR0 */
    {0, 1}, /* CT16B0, MR1 */
 
    {1, 0}, /* CT16B1, MR0 */
    {1, 1}, /* CT16B1, MR1 */
 
    {2, 2}, /* CT32B0, MR2 */
};

static LPC_TMR_TypeDef *Timers[3] = {
    LPC_TMR16B0, LPC_TMR16B1,
    LPC_TMR32B0
};
#endif


void FastPWM::initFastPWM( void ) {
    fast_obj = new fastpwm_struct;
    timer_mr tid = pwm_timer_map[_pwm.pwm];
    PWM_TIMER = Timers[tid.timer];
    (((fastpwm_struct*)fast_obj)->MR) = &PWM_TIMER->MR[tid.mr];
    
    if (tid.timer < 2)
        //16-bit timer
        bits = 16;
    else
        //32-bit timer
        bits = 32;  
}

void FastPWM::pulsewidth_ticks( uint32_t ticks ) {
    if (ticks)
        PWM_MR = PWM_TIMER->MR3 - ticks;  //They inverted PWM on the 11u24
    else
        PWM_MR = 0xFFFFFFFF;           //If MR3 = ticks 1 clock cycle wide errors appear, this prevents that (unless MR3 = max).
}

void FastPWM::period_ticks( uint32_t ticks ) {  
    PWM_TIMER->TCR = 0x02;
    PWM_TIMER->MR3 = ticks;
    PWM_TIMER->TCR = 0x01;   
}

uint32_t FastPWM::getPeriod( void ) {
    return PWM_TIMER->MR3;
}

uint32_t FastPWM::setPrescaler(uint32_t reqScale) {
    //If 32-bit, disable auto-scaling, return 1
    if (bits == 32) {
        dynamicPrescaler = false;
        return 1;
    }
    
    //Else 16-bit timer:
    if (reqScale == 0)
        //Return prescaler
        return PWM_TIMER->PR + 1;
    if (reqScale > (uint32_t)(1<<16))
        reqScale = 1<<16;
    //Else set prescaler, we have to substract one from reqScale since a 0 in PCVAL is prescaler of 1
    PWM_TIMER->PR = reqScale - 1;

    return reqScale;
}

#endif