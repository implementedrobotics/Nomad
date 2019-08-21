#ifndef MOTOR_CONFIG_H
#define MOTOR_CONFIG_H

#define R_PHASE 0.04271116107702255f //Ohms
#define L_D 0.00001578306f           //Henries
#define L_Q 0.00001578306f           //Henries
#define KT 0.029538461538f           //N-m per peak phase amp, = WB*NPP*3/2
#define NPP 7                        //Number of pole pairs
#define GR 1.0f                      //Gear ratio
#define KT_OUT 0.029538461538f       //KT*GR
#define WB 0.002812902528f                   //Flux linkage, Webers.  (60/(SQRT(3) * KV* PI * NPP * 2))
#define R_TH 1.25f                   //Kelvin per watt
#define INV_M_TH 0.03125f            //Kelvin per joule 




#endif
