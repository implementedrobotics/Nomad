
#include "mbed.h"
#include "PositionSensor.h"
#include "../math_ops.h"
//#include "offset_lut.h"
//#include <math.h>

PositionSensorAM5147::PositionSensorAM5147(int CPR, float offset, int ppairs){
    //_CPR = CPR;
    _CPR = CPR;
    _ppairs = ppairs;
    ElecOffset = offset;
    rotations = 0;
    spi = new SPI(PC_12, PC_11, PC_10);
    spi->format(16, 1);                                                          // mbed v>127 breaks 16-bit spi, so transaction is broken into 2 8-bit words
    spi->frequency(25000000);
    
    cs = new DigitalOut(PA_15);
    cs->write(1);
    readAngleCmd = 0xffff;   
    MechOffset = offset;
    modPosition = 0;
    oldModPosition = 0;
    oldVel = 0;
    raw = 0;
    }
    
void PositionSensorAM5147::Sample(float dt){
    GPIOA->ODR &= ~(1 << 15); // Pull Low (Should be able to use a GPIO "pin" for this)
    raw = spi->write(readAngleCmd);
    raw &= 0x3FFF;   
    //raw = spi->write(0);
    raw = raw>>2;                                                             //Extract last 14 bits
    GPIOA->ODR |= (1 << 15); // Pull High
    int off_1 = offset_lut[raw>>7];
    int off_2 = offset_lut[((raw>>7)+1)%128];
    int off_interp = off_1 + ((off_2 - off_1)*(raw - ((raw>>7)<<7))>>7);        // Interpolate between lookup table entries
    int angle = raw + off_interp;                                               // Correct for nonlinearity with lookup table from calibration
    if(angle - old_counts > _CPR/2){
        rotations -= 1;
        }
    else if (angle - old_counts < -_CPR/2){
        rotations += 1;
        }
    
    old_counts = angle;
    oldModPosition = modPosition;
    modPosition = ((2.0f*PI * ((float) angle))/ (float)_CPR);
    position = (2.0f*PI * ((float) angle+(_CPR*rotations)))/ (float)_CPR;
    MechPosition = position - MechOffset;
    float elec = ((2.0f*PI/(float)_CPR) * (float) ((_ppairs*angle)%_CPR)) + ElecOffset;
    if(elec < 0) elec += 2.0f*PI;
    else if(elec > 2.0f*PI) elec -= 2.0f*PI ; 
    ElecPosition = elec;
    
    float vel;
    //if(modPosition<.1f && oldModPosition>6.1f){

    if((modPosition-oldModPosition) < -3.0f){
        vel = (modPosition - oldModPosition + 2.0f*PI)/dt;
        }
    //else if(modPosition>6.1f && oldModPosition<0.1f){
    else if((modPosition - oldModPosition) > 3.0f){
        vel = (modPosition - oldModPosition - 2.0f*PI)/dt;
        }
    else{
        vel = (modPosition-oldModPosition)/dt;
    }    
    
    int n = 40;
    float sum = vel;
    for (int i = 1; i < (n); i++){
        velVec[n - i] = velVec[n-i-1];
        sum += velVec[n-i];
        }
    velVec[0] = vel;
    MechVelocity =  sum/((float)n);
    ElecVelocity = MechVelocity*_ppairs;
    ElecVelocityFilt = 0.99f*ElecVelocityFilt + 0.01f*ElecVelocity;
    }

int PositionSensorAM5147::GetRawPosition(){
    return raw;
    }

float PositionSensorAM5147::GetMechPositionFixed(){
    return MechPosition+MechOffset;
    }
    
float PositionSensorAM5147::GetMechPosition(){
    return MechPosition;
    }

float PositionSensorAM5147::GetElecPosition(){
    return ElecPosition;
    }

float PositionSensorAM5147::GetElecVelocity(){
    return ElecVelocity;
    }

float PositionSensorAM5147::GetMechVelocity(){
    return MechVelocity;
    }

void PositionSensorAM5147::ZeroPosition(){
    rotations = 0;
    MechOffset = 0;
    Sample(.00025f);
    MechOffset = GetMechPosition();
    }
    
void PositionSensorAM5147::SetElecOffset(float offset){
    ElecOffset = offset;
    }
void PositionSensorAM5147::SetMechOffset(float offset){
    MechOffset = offset;
    }

int PositionSensorAM5147::GetCPR(){
    return _CPR;
    }


void PositionSensorAM5147::WriteLUT(int new_lut[128]){
    memcpy(offset_lut, new_lut, sizeof(offset_lut));
    }
    


PositionSensorEncoder::PositionSensorEncoder(int CPR, float offset, int ppairs) {
    _ppairs = ppairs;
    _CPR = CPR;
    _offset = offset;
    MechPosition = 0;
    out_old = 0;
    oldVel = 0;
    raw = 0;
    
    // Enable clock for GPIOA
    __GPIOA_CLK_ENABLE(); //equivalent from hal_rcc.h
 
    GPIOA->MODER   |= GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1 ;           //PA6 & PA7 as Alternate Function   /*!< GPIO port mode register,               Address offset: 0x00      */
    GPIOA->OTYPER  |= GPIO_OTYPER_OT_6 | GPIO_OTYPER_OT_7 ;                 //PA6 & PA7 as Inputs               /*!< GPIO port output type register,        Address offset: 0x04      */
    GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR6 | GPIO_OSPEEDER_OSPEEDR7 ;     //Low speed                         /*!< GPIO port output speed register,       Address offset: 0x08      */
    GPIOA->PUPDR   |= GPIO_PUPDR_PUPDR6_1 | GPIO_PUPDR_PUPDR7_1 ;           //Pull Down                         /*!< GPIO port pull-up/pull-down register,  Address offset: 0x0C      */
    GPIOA->AFR[0]  |= 0x22000000 ;                                          //AF02 for PA6 & PA7                /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
    GPIOA->AFR[1]  |= 0x00000000 ;                                          //nibbles here refer to gpio8..15   /*!< GPIO alternate function registers,     Address offset: 0x20-0x24 */
   
    // configure TIM3 as Encoder input
    // Enable clock for TIM3
    __TIM3_CLK_ENABLE();
 
    TIM3->CR1   = 0x0001;                                                   // CEN(Counter ENable)='1'     < TIM control register 1
    TIM3->SMCR  = TIM_ENCODERMODE_TI12;                                     // SMS='011' (Encoder mode 3)  < TIM slave mode control register
    TIM3->CCMR1 = 0x1111;                                                   // CC1S='01' CC2S='01'         < TIM capture/compare mode register 1, maximum digital filtering
    TIM3->CCMR2 = 0x0000;                                                   //                             < TIM capture/compare mode register 2
    TIM3->CCER  = 0x0011;                                                   // CC1P CC2P                   < TIM capture/compare enable register
    TIM3->PSC   = 0x0000;                                                   // Prescaler = (0+1)           < TIM prescaler
    TIM3->ARR   = CPR;                                                      // IM auto-reload register
  
    TIM3->CNT = 0x000;  //reset the counter before we use it  
    
    // Extra Timer for velocity measurement
    
    __TIM2_CLK_ENABLE();
    TIM3->CR2 = 0x030;                                                      //MMS = 101
    
    TIM2->PSC = 0x03;
    //TIM2->CR2 |= TIM_CR2_TI1S;
    TIM2->SMCR = 0x24;                                                      //TS = 010 for ITR2, SMS = 100 (reset counter at edge)
    TIM2->CCMR1 = 0x3;                                                      // CC1S = 11, IC1 mapped on TRC
    
    //TIM2->CR2 |= TIM_CR2_TI1S;
    TIM2->CCER |= TIM_CCER_CC1P;
    //TIM2->CCER |= TIM_CCER_CC1NP;
    TIM2->CCER |= TIM_CCER_CC1E;
    
    
    TIM2->CR1 = 0x01;                                                       //CEN,  enable timer
    
    TIM3->CR1   = 0x01;                                                     // CEN
    ZPulse = new InterruptIn(PC_4);
    ZSense = new DigitalIn(PC_4);
    //ZPulse = new InterruptIn(PB_0);
    //ZSense = new DigitalIn(PB_0);
    ZPulse->enable_irq();
    ZPulse->rise(this, &PositionSensorEncoder::ZeroEncoderCount);
    //ZPulse->fall(this, &PositionSensorEncoder::ZeroEncoderCountDown);
    ZPulse->mode(PullDown);
    flag = 0;

    
    //ZTest = new DigitalOut(PC_2);
    //ZTest->write(1);
    }
    
void PositionSensorEncoder::Sample(float dt){
    
    }

 
float PositionSensorEncoder::GetMechPosition() {                            //returns rotor angle in radians.
    int raw = TIM3->CNT;
    float unsigned_mech = (6.28318530718f/(float)_CPR) * (float) ((raw)%_CPR);
    return (float) unsigned_mech;// + 6.28318530718f* (float) rotations;
}

float PositionSensorEncoder::GetElecPosition() {                            //returns rotor electrical angle in radians.
    int raw = TIM3->CNT;
    float elec = ((6.28318530718f/(float)_CPR) * (float) ((_ppairs*raw)%_CPR)) - _offset;
    if(elec < 0) elec += 6.28318530718f;
    return elec;
}


    
float PositionSensorEncoder::GetMechVelocity(){

    float out = 0;
    float rawPeriod = TIM2->CCR1; //Clock Ticks
    int currentTime = TIM2->CNT;
    if(currentTime > 2000000){rawPeriod = currentTime;}
    float  dir = -2.0f*(float)(((TIM3->CR1)>>4)&1)+1.0f;    // +/- 1
    float meas = dir*180000000.0f*(6.28318530718f/(float)_CPR)/rawPeriod; 
    if(isinf(meas)){ meas = 1;}
    out = meas;
    //if(meas == oldVel){
     //   out = .9f*out_old;
     //   }
    
 
    oldVel = meas;
    out_old = out;
    int n = 16;
    float sum = out;
    for (int i = 1; i < (n); i++){
        velVec[n - i] = velVec[n-i-1];
        sum += velVec[n-i];
        }
    velVec[0] = out;
    return sum/(float)n;
    }
    
float PositionSensorEncoder::GetElecVelocity(){
    return _ppairs*GetMechVelocity();
    }
    
void PositionSensorEncoder::ZeroEncoderCount(void){
    if (ZSense->read() == 1 & flag == 0){
        if (ZSense->read() == 1){
            GPIOC->ODR ^= (1 << 4);   
            TIM3->CNT = 0x000;
            //state = !state;
            //ZTest->write(state);
            GPIOC->ODR ^= (1 << 4);
            //flag = 1;
        }
        }
    }

void PositionSensorEncoder::ZeroPosition(void){
    
    }
    
void PositionSensorEncoder::ZeroEncoderCountDown(void){
    if (ZSense->read() == 0){
        if (ZSense->read() == 0){
            GPIOC->ODR ^= (1 << 4);
            flag = 0;
            float dir = -2.0f*(float)(((TIM3->CR1)>>4)&1)+1.0f;
            if(dir != dir){
                dir = dir;
                rotations +=  dir;
                }

            GPIOC->ODR ^= (1 << 4);

        }
        }
    }
void PositionSensorEncoder::SetElecOffset(float offset){
    
    }
    
int PositionSensorEncoder::GetRawPosition(void){
    return 0;
    }
    
int PositionSensorEncoder::GetCPR(){
    return _CPR;
    }
    

void PositionSensorEncoder::WriteLUT(int new_lut[128]){
    memcpy(offset_lut, new_lut, sizeof(offset_lut));
    }
