#ifndef CURRENT_CONTROLLER_CONFIG_H
#define CURRENT_CONTROLLER_CONFIG_H

// Current controller///
#define K_D .05f                    // Loop gain,  Volts/Amp
#define K_Q .05f                    // Loop gain,  Volts/Amp
#define K_SCALE 0.0001f             // K_loop/Loop BW (Hz) 0.0042
#define KI_D 0.0255f                // PI zero, in radians per sample
#define KI_Q 0.0255f                // PI zero, in radians per sample
#define V_BUS 24.0f                 // Volts
#define OVERMODULATION 1.15f        // 1.0 = no overmodulation

#define D_INT_LIM V_BUS/(K_D*KI_D)  // Amps*samples
#define Q_INT_LIM V_BUS/(K_Q*KI_Q)  // Amps*samples

#define I_MAX 40.0f                 // Max Current
#define I_MAX_FW 10.0f               // Max field weakening current
#define I_MAX_CONT 15.0f            // Max continuous current, for thermal limiting

//Observer//
#define DT 0.000025f
#define K_O 0.02f




#endif
