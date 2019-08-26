/*
 * Motor.h
 *
 *  Created on: August 24, 2019
 *      Author: Quincy Jones
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

#ifndef CORE_MOTOR_H_
#define CORE_MOTOR_H_

class Motor {

public:

    // Motor State
    struct State_t {
        float I_a;      // Phase A Currents
        float I_b;      // Phase B Currents
        float I_c;      // Phase C Currents
        float theta_mech;      // Mechanical Position @ Output (Radians)
        float theta_mech_dot;  // Mechanical Velocity @ Output (Radians/Sec)
        float theta_elec;      // Electrical Position @ Rotor (Radians)
        float theta_elec_dot;  // Electrical Velocity @ Rotor (Radians/Sec)
        float windings_temp;   // Motor Windings Temperature (Degrees Celcius)
    }

    // Motor Parameters
    struct Param_t {
        uint32_t num_pole_pairs; // Pole Pairs of Motor (PAIRS)
        float phase_resistance;  // Phase Resistance (Ohms)
        float phase_inductance_d; // D Axis Phase Inductance (Henries)
        float phase_inductance_q; // Q Axis Phase Inductance (Henries)
        float K_v;                // Motor KV Rating (V/RPM)
        float flux_linkage;       // Rotor Flux Linkage (Webers)
        float K_t;                // Torque Constant (N*m/A)
    };

    // TODO: Flash, Indexing Etc to save

    Motor();
private:
    Param_t param_; // Motor Params
    State_t state_; // Motor State


};

#endif // CORE_MOTOR_H_