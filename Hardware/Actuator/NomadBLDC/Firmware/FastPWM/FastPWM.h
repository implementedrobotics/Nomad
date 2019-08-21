/*
    .---.           _....._
   /  p  `\     .-""`:     :`"-.
   |__   - |  ,'     .     '    ',
    ._>    \ /:      :     ;      :,
     '-.    '\`.     .     :     '  \
        `.   | .'._.' '._.' '._.'.  |
          `;-\.   :     :     '   '/,__,
          .-'`'._ '     .     : _.'.__.'
         ((((-'/ `";--..:..--;"` \
             .'   /           \   \
       jgs  ((((-'           ((((-'
       
Yeah ASCII art turtle more fun than copyright stuff
*/


#include "mbed.h"

#ifndef FASTPWM_H
#define FASTPWM_H

/** Library that allows faster and/or higher resolution PWM output
  *
  * Library can directly replace standard mbed PWM library.
  *
  * Contrary to the default mbed library, this library takes doubles instead of floats. The compiler will autocast if needed,
  * but do take into account it is done for a reason, your accuracy will otherwise be limitted by the floating point precision.
  */
class FastPWM : public PwmOut {
public:
    /**
    * Create a FastPWM object connected to the specified pin
    *
    * @param pin - PWM pin to connect to
    * @param prescaler - Clock prescaler, -1 is dynamic (default), 0 is bit random, everything else normal
    */
    FastPWM(PinName pin, int prescaler = -1);
    ~FastPWM(); 
    
    /**
    * Set the PWM period, specified in seconds (double), keeping the pulsewidth the same.
    */
    void period(double seconds);
    
    /**
    * Set the PWM period, specified in milli-seconds (int), keeping the pulsewidth the same.
    */
    void period_ms(int ms);
    
    /**
    * Set the PWM period, specified in micro-seconds (int), keeping the pulsewidth the same.
    */
    void period_us(int us);
    
    /**
    * Set the PWM period, specified in micro-seconds (double), keeping the pulsewidth the same.
    */
    void period_us(double us);
    
    /**
    * Set the PWM period, specified in clock ticks, keeping _pulse width_ the same.
    *
    * This function can be used if low overhead is required. Do take into account the result is
    * board (clock frequency) dependent, and this does not keep an equal duty cycle!
    */
    void period_ticks(uint32_t ticks);
    
    /**
    * Set the PWM pulsewidth, specified in seconds (double), keeping the period the same.
    */
    void pulsewidth(double seconds);
    
    /**
    * Set the PWM pulsewidth, specified in milli-seconds (int), keeping the period the same.
    */
    void pulsewidth_ms(int ms);
    
    /**
    * Set the PWM pulsewidth, specified in micro-seconds (int), keeping the period the same.
    */
    void pulsewidth_us(int us);
    
    /**
    * Set the PWM pulsewidth, specified in micro-seconds (double), keeping the period the same.
    */
    void pulsewidth_us(double us);
    
    /**
    * Set the PWM period, specified in clock ticks, keeping the period the same.
    *
    * This function can be used if low overhead is required. Do take into account the result is
    * board (clock frequency) dependent!
    */
    void pulsewidth_ticks(uint32_t ticks);
    
    /**
    * Set the ouput duty-cycle, specified as a percentage (double)
    *
    * @param duty - A double value representing the output duty-cycle, specified as a percentage.  The value should lie between 0.0 (representing on 0%) and 1.0 (representing on 100%).
    */
    void write(double duty);
    
    /**
    * Return the ouput duty-cycle, specified as a percentage (double)
    *
    * @param return - A double value representing the output duty-cycle, specified as a percentage.
    */
    double read( void );
    
    /**
    * An operator shorthand for write()
    */
    FastPWM& operator= (double value);
    
    /**
    * An operator shorthand for read()
    */
    operator double();
    
    /**
    * Set the PWM prescaler
    *
    * The period of all PWM pins on the same PWM unit have to be reset after using this!
    *
    * @param value - The required prescaler. Special values: 0 = lock current prescaler, -1 = use dynamic prescaler
    * @param return - The prescaler which was set (can differ from requested prescaler if not possible)
    */
    int prescaler(int value);
    
private:
    void initFastPWM(void);
    
    uint32_t setPrescaler( uint32_t reqScale );
    int calcPrescaler(uint64_t clocks);
    uint32_t getPeriod( void );
    
    void updateTicks( uint32_t prescaler );
    uint32_t bits;
    
    double _duty;
    
    double dticks, dticks_us;
    int iticks_ms, iticks_us;
    
    bool dynamicPrescaler;
    
    void *fast_obj;
};
#endif