/*
 * main.cpp
 *
 *  Created on: August 21, 2019
 *      Original Work Written By: Ben Katz, with much inspiration from Bayley Wang, 
 *      Nick Kirkby, Shane Colton, David Otten, and others.
 *      Original Documentation at: http://build-its.blogspot.com/
 *      Forked From: https://os.mbed.com/users/benkatz/code/Hobbyking_Cheetah_Compact/
 *      Original License: Unspecified/Unknown
 * 
 *      Modifications By: Quincy Jones
 *
 * Copyright (c) <2019> <Quincy Jones - quincy@implementedrobotics.com/>
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software
 * is furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
 * WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 * 
 */

//

/// high-bandwidth 3-phase motor control, for robots
/// Written by benkatz, with much inspiration from Bayley Wang, Nick Kirkby, Shane Colton, David Otten, and others
/// Hardware documentation can be found at build-its.blogspot.com
/// Written for the STM32F446, but can be implemented on other STM32 MCU's with some further register-diddling
/// Version for the TI DRV8323 Everything Chip

#define REST_MODE 0
#define CALIBRATION_MODE 1
#define MOTOR_MODE 2
#define SETUP_MODE 4
#define ENCODER_MODE 5

#define VERSION_NUM "0.1"


float __float_reg[64];                                                          // Floats stored in flash
int __int_reg[256];                                                             // Ints stored in flash.  Includes position sensor calibration lookup table

#include "mbed.h"
#include "PositionSensor.h"
#include "structs.h"
#include "foc.h"
#include "calibration.h"
#include "hw_setup.h"
#include "math_ops.h" 
#include "current_controller_config.h"
#include "hw_config.h"
#include "motor_config.h"
#include "stm32f4xx_flash.h"
#include "FlashWriter.h"
#include "user_config.h"
#include "PreferenceWriter.h"
#include "CAN_com.h"
#include "DRV.h"
 
PreferenceWriter prefs(6);

GPIOStruct gpio;
ControllerStruct controller;
ObserverStruct observer;
COMStruct com;
Serial pc(PA_2, PA_3);


CAN          can(PB_8, PB_9, 1000000);      // CAN Rx pin name, CAN Tx pin name
CANMessage   rxMsg;
CANMessage   txMsg;


SPI drv_spi(PA_7, PA_6, PA_5);
DigitalOut drv_cs(PA_4);
//DigitalOut drv_en_gate(PA_11);
DRV832x drv(&drv_spi, &drv_cs);

PositionSensorAM5147 spi(16384, 0.0, NPP);  

volatile int count = 0;
volatile int state = REST_MODE;
volatile int state_change;

void onMsgReceived() {
    //msgAvailable = true;
    printf("%df\n\r", rxMsg.id);
    can.read(rxMsg);  
    if((rxMsg.id == CAN_ID)){
        controller.timeout = 0;
        if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) & (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFC))){
            state = MOTOR_MODE;
            state_change = 1;
            }
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFD))){
            state = REST_MODE;
            state_change = 1;
            gpio.led->write(0);; 
            }
        else if(((rxMsg.data[0]==0xFF) & (rxMsg.data[1]==0xFF) & (rxMsg.data[2]==0xFF) & (rxMsg.data[3]==0xFF) * (rxMsg.data[4]==0xFF) & (rxMsg.data[5]==0xFF) & (rxMsg.data[6]==0xFF) & (rxMsg.data[7]==0xFE))){
            spi.ZeroPosition();
            }
        else if(state == MOTOR_MODE){
            unpack_cmd(rxMsg, &controller);
            }
        pack_reply(&txMsg, controller.theta_mech, controller.dtheta_mech, controller.i_q_filt*KT_OUT);
        can.write(txMsg);
        }
    
}

void enter_menu_state(void){
    drv.disable_gd();
    //gpio.enable->write(0);
    printf("\n\r\n\r\n\r");
    printf(" Commands New:\n\r");
    wait_us(10);
    printf(" m - Motor Mode\n\r");
    wait_us(10);
    printf(" c - Calibrate Encoder\n\r");
    wait_us(10);
    printf(" s - Setup\n\r");
    wait_us(10);
    printf(" e - Display Encoder\n\r");
    wait_us(10);
    printf(" z - Set Zero Position\n\r");
    wait_us(10);
    printf(" esc - Exit to Menu\n\r");
    wait_us(10);
    state_change = 0;
    gpio.led->write(0);
    }

void enter_setup_state(void){
    printf("\n\r\n\r Configuration Options \n\r\n\n");
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-5s\n\r\n\r", "prefix", "parameter", "min", "max", "current value");
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "b", "Current Bandwidth (Hz)", "100", "2000", I_BW);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "i", "CAN ID", "0", "127", CAN_ID);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %-5i\n\r", "m", "CAN Master ID", "0", "127", CAN_MASTER);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %.1f\n\r", "l", "Torque Limit (N-m)", "0.0", "18.0", TORQUE_LIMIT);
    wait_us(10);
    printf(" %-4s %-31s %-5s %-6s %d\n\r", "t", "CAN Timeout (cycles)(0 = none)", "0", "100000", CAN_TIMEOUT);
    wait_us(10);
    printf("\n\r To change a value, type 'prefix''value''ENTER'\n\r i.e. 'b1000''ENTER'\n\r\n\r");
    wait_us(10);
    state_change = 0;
    }
    
void enter_torque_mode(void){
    drv.enable_gd();
    //gpio.enable->write(1);
    controller.ovp_flag = 0;
    reset_foc(&controller);                                                     // Tesets integrators, and other control loop parameters
    wait(.001);
    controller.i_d_ref = 0;
    controller.i_q_ref = 0;                                                     // Current Setpoints
    gpio.led->write(1);                                                     // Turn on status LED
    state_change = 0;
    printf("\n\r Entering Motor Mode \n\r");
    }
    
void calibrate(void){
    drv.enable_gd();
    //gpio.enable->write(1);
    gpio.led->write(1);                                                    // Turn on status LED
    order_phases(&spi, &gpio, &controller, &prefs);                             // Check phase ordering
    calibrate(&spi, &gpio, &controller, &prefs);                                // Perform calibration procedure
    gpio.led->write(0);;                                                     // Turn off status LED
    wait(.2);
    printf("\n\r Calibration complete.  Press 'esc' to return to menu\n\r");
    drv.disable_gd();
    //gpio.enable->write(0);
     state_change = 0;
    }
    
void print_encoder(void){
    printf(" Mechanical Angle:  %f    Electrical Angle:  %f    Raw:  %d\n\r", spi.GetMechPosition(), spi.GetElecPosition(), spi.GetRawPosition());
    //printf("%d\n\r", spi.GetRawPosition());
    wait(.001);
    }

/// Current Sampling Interrupt ///
/// This runs at 40 kHz, regardless of of the mode the controller is in ///
extern "C" void TIM1_UP_TIM10_IRQHandler(void) {
  if (TIM1->SR & TIM_SR_UIF ) {

        ///Sample current always ///
        ADC1->CR2  |= 0x40000000;                                               // Begin sample and conversion
        //volatile int delay;   
        //for (delay = 0; delay < 55; delay++);

        spi.Sample(DT);                                                           // sample position sensor
        controller.adc2_raw = ADC2->DR;                                         // Read ADC Data Registers
        controller.adc1_raw = ADC1->DR;
        controller.adc3_raw = ADC3->DR;
        controller.theta_elec = spi.GetElecPosition();
        controller.theta_mech = (1.0f/GR)*spi.GetMechPosition();
        controller.dtheta_mech = (1.0f/GR)*spi.GetMechVelocity();  
        controller.dtheta_elec = spi.GetElecVelocity();
        controller.v_bus = 0.95f*controller.v_bus + 0.05f*((float)controller.adc3_raw)*V_SCALE;
        ///
        
        /// Check state machine state, and run the appropriate function ///
        switch(state){
            case REST_MODE:                                                     // Do nothing
                if(state_change){
                    enter_menu_state();
                    }
                break;
            
            case CALIBRATION_MODE:                                              // Run encoder calibration procedure
                if(state_change){
                    calibrate();
                    }
                break;
             
            case MOTOR_MODE:                                                   // Run torque control
                if(state_change){
                    enter_torque_mode();
                    count = 0;
                    }
                else{
                /*
                if(controller.v_bus>28.0f){         //Turn of gate drive if bus voltage is too high, to prevent FETsplosion if the bus is cut during regen
                    gpio.
                    ->write(0);
                    controller.ovp_flag = 1;
                    state = REST_MODE;
                    state_change = 1;
                    printf("OVP Triggered!\n\r");
                    }
                    */  

                if((controller.timeout > CAN_TIMEOUT) && (CAN_TIMEOUT > 0)){
                    controller.i_d_ref = 0;
                    controller.i_q_ref = 0;
                    controller.kp = 0;
                    controller.kd = 0;
                    controller.t_ff = 0;
                    } 

                torque_control(&controller);
                commutate(&controller, &observer, &gpio, controller.theta_elec);           // Run current loop

                controller.timeout++;
                count++; 
            
                }     
                break;
            case SETUP_MODE:
                if(state_change){
                    enter_setup_state();
                }
                break;
            case ENCODER_MODE:
                print_encoder();
                break;
                }                 
      }
  TIM1->SR = 0x0;                                                               // reset the status register
}


char cmd_val[8] = {0};
char cmd_id = 0;
char char_count = 0;

/// Manage state machine with commands from serial terminal or configurator gui ///
/// Called when data received over serial ///
void serial_interrupt(void){
    while(pc.readable()){
        char c = pc.getc();
        if(c == 27){
                state = REST_MODE;
                state_change = 1;
                char_count = 0;
                cmd_id = 0;
                gpio.led->write(0);; 
                for(int i = 0; i<8; i++){cmd_val[i] = 0;}
                }
        if(state == REST_MODE){
            switch (c){
                case 'c':
                    state = CALIBRATION_MODE;
                    state_change = 1;
                    break;
                case 'm':
                    state = MOTOR_MODE;
                    state_change = 1;
                    break;
                case 'e':
                    state = ENCODER_MODE;
                    state_change = 1;
                    break;
                case 's':
                    state = SETUP_MODE;
                    state_change = 1;
                    break;
                case 'z':
                    spi.SetMechOffset(0);
                    spi.Sample(DT);
                    wait_us(20);
                    M_OFFSET = spi.GetMechPosition();
                    if (!prefs.ready()) prefs.open();
                        prefs.flush();                                                  // Write new prefs to flash
                        prefs.close();    
                        prefs.load(); 
                    spi.SetMechOffset(M_OFFSET);
                    printf("\n\r  Saved new zero position:  %.4f\n\r\n\r", M_OFFSET);
                    
                    break;
                }
                
                }
        else if(state == SETUP_MODE){
            if(c == 13){
                switch (cmd_id){
                    case 'b':
                        I_BW = fmaxf(fminf(atof(cmd_val), 2000.0f), 100.0f);
                        break;
                    case 'i':
                        CAN_ID = atoi(cmd_val);
                        break;
                    case 'm':
                        CAN_MASTER = atoi(cmd_val);
                        break;
                    case 'l':
                        TORQUE_LIMIT = fmaxf(fminf(atof(cmd_val), 18.0f), 0.0f);
                        break;
                    case 't':
                        CAN_TIMEOUT = atoi(cmd_val);
                        break;
                    default:
                        printf("\n\r '%c' Not a valid command prefix\n\r\n\r", cmd_id);
                        break;
                    }
                    
                if (!prefs.ready()) prefs.open();
                prefs.flush();                                                  // Write new prefs to flash
                prefs.close();    
                prefs.load();                                              
                state_change = 1;
                char_count = 0;
                cmd_id = 0;
                for(int i = 0; i<8; i++){cmd_val[i] = 0;}
                }
            else{
                if(char_count == 0){cmd_id = c;}
                else{
                    cmd_val[char_count-1] = c;
                    
                }
                pc.putc(c);
                char_count++;
                }
            }
        else if (state == ENCODER_MODE){
            switch (c){
                case 27:
                    state = REST_MODE;
                    state_change = 1;
                    break;
                    }
            }
        else if (state == MOTOR_MODE){
            switch (c){
                case 'd':
                    controller.i_q_ref = 0;
                    controller.i_d_ref = 0;
                }
            }
            
        }
    }
       
int main() {
    controller.v_bus = V_BUS;
    controller.mode = 0;
    Init_All_HW(&gpio);                                                         // Setup PWM, ADC, GPIO
    wait(.1);
    
    gpio.enable->write(1);
    wait_us(100);
    drv.calibrate();
    wait_us(100);
    drv.write_DCR(0x0, 0x0, 0x0, PWM_MODE_3X, 0x0, 0x0, 0x0, 0x0, 0x1);
    wait_us(100);
    drv.write_CSACR(0x0, 0x1, 0x0, CSA_GAIN_40, 0x0, 0x0, 0x0, 0x0, SEN_LVL_1_0);
    wait_us(100);
    drv.write_OCPCR(TRETRY_4MS, DEADTIME_200NS, OCP_RETRY, OCP_DEG_8US, VDS_LVL_1_88);

    int val = drv.read_register(DCR);
    
    
    //drv.enable_gd();
    zero_current(&controller.adc1_offset, &controller.adc2_offset);             // Measure current sensor zero-offset
    drv.disable_gd();
    
    wait(.1);
    /*
    gpio.enable->write(1);
    TIM1->CCR3 = 0x708*(1.0f);                        // Write duty cycles
    TIM1->CCR2 = 0x708*(1.0f);
    TIM1->CCR1 = 0x708*(1.0f);
    gpio.enable->write(0);
    */
    reset_foc(&controller);                                                     // Reset current controller
    reset_observer(&observer);                                                 // Reset observer
    TIM1->CR1 ^= TIM_CR1_UDIS;
    //TIM1->CR1 |= TIM_CR1_UDIS; //enable interrupt
    
    wait(.1);
    NVIC_SetPriority(TIM1_UP_TIM10_IRQn, 2);                                             // commutation > communication
    
    NVIC_SetPriority(CAN1_RX0_IRQn, 3);
    can.filter(CAN_ID<<21, 0xFFE00004, CANStandard, 0);
                                                                    
    txMsg.id = CAN_MASTER;
    txMsg.len = 6;
    rxMsg.len = 8;
    can.attach(&onMsgReceived);                                     // attach 'CAN receive-complete' interrupt handler    
    
    // If preferences haven't been user configured yet, set defaults 
    prefs.load();                                                               // Read flash
    if(isnan(E_OFFSET)){E_OFFSET = 0.0f;}
    if(isnan(M_OFFSET)){M_OFFSET = 0.0f;}
    if(isnan(I_BW) || I_BW==-1){I_BW = 1000;}
    if(isnan(TORQUE_LIMIT) || TORQUE_LIMIT ==-1){TORQUE_LIMIT=18;}
    if(isnan(CAN_ID) || CAN_ID==-1){CAN_ID = 1;}
    if(isnan(CAN_MASTER) || CAN_MASTER==-1){CAN_MASTER = 0;}
    if(isnan(CAN_TIMEOUT) || CAN_TIMEOUT==-1){CAN_TIMEOUT = 0;}
    spi.SetElecOffset(E_OFFSET);                                                // Set position sensor offset
    spi.SetMechOffset(M_OFFSET);
    int lut[128] = {0};
    memcpy(&lut, &ENCODER_LUT, sizeof(lut));
    spi.WriteLUT(lut);                                                          // Set potision sensor nonlinearity lookup table
    init_controller_params(&controller);

    pc.baud(230400);                                                            // set serial baud rate
    //wait(10);
    //printf(" DCR:  %d\n\r", val);
    pc.printf("\n\r\n\r Nomad BLDC\n\r\n\r");
    wait(.01);
    printf("\n\r Debug Info:\n\r");
    printf(" Firmware Version: %s\n\r", VERSION_NUM);
    printf(" ADC1 Offset: %d    ADC2 Offset: %d\n\r", controller.adc1_offset, controller.adc2_offset);
    printf(" Position Sensor Electrical Offset:   %.4f\n\r", E_OFFSET);
    printf(" Output Zero Position:  %.4f\n\r", M_OFFSET);
    printf(" CAN ID:  %d\n\r", CAN_ID);
    



    printf(" %d\n\r", drv.read_register(DCR));
    wait_us(100);
    printf(" %d\n\r", drv.read_register(CSACR));
    wait_us(100);
    printf(" %d\n\r", drv.read_register(OCPCR));
    drv.disable_gd();
    
    pc.attach(&serial_interrupt);                                               // attach serial interrupt
    
    state_change = 1;


    int counter = 0;
    while(1) {
        //drv.print_faults();
       wait(.1);
       //printf("%.4f\n\r", controller.v_bus);
       //gpio.led->write(1); 
       
        if(state == MOTOR_MODE)
        {
            printf("%.3f  %.3f  %.3f\n\r", (float)observer.temperature, (float)observer.temperature2, observer.resistance);
            printf("%.3f  %.3f  %.3f %.3f %.3f\n\r", controller.v_d, controller.v_q, controller.i_d_filt, controller.i_q_filt, controller.dtheta_elec);
            printf("%.3f\n\r", controller.dtheta_mech);
            wait(.002);
        }
       
    
    }
}
