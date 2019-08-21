
#include "foc.h"
using namespace FastMath;

void abc(float theta, float d, float q, float *a, float *b, float *c)
{
    /// Inverse DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///
    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *a = cf * d - sf * q; // Faster Inverse DQ0 transform
    *b = (0.86602540378f * sf - .5f * cf) * d - (-0.86602540378f * cf - .5f * sf) * q;
    *c = (-0.86602540378f * sf - .5f * cf) * d - (0.86602540378f * cf - .5f * sf) * q;
}

void dq0(float theta, float a, float b, float c, float *d, float *q)
{
    /// DQ0 Transform ///
    ///Phase current amplitude = lengh of dq vector///
    ///i.e. iq = 1, id = 0, peak phase current of 1///

    float cf = FastCos(theta);
    float sf = FastSin(theta);

    *d = 0.6666667f * (cf * a + (0.86602540378f * sf - .5f * cf) * b + (-0.86602540378f * sf - .5f * cf) * c); ///Faster DQ0 Transform
    *q = 0.6666667f * (-sf * a - (-0.86602540378f * cf - .5f * sf) * b - (0.86602540378f * cf - .5f * sf) * c);
}

void svm(float v_bus, float u, float v, float w, float *dtc_u, float *dtc_v, float *dtc_w)
{
    /// Space Vector Modulation ///
    /// u,v,w amplitude = v_bus for full modulation depth ///

    float v_offset = (fminf3(u, v, w) + fmaxf3(u, v, w)) * 0.5f;

    *dtc_u = fminf(fmaxf(((u - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf(((v - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf(((w - v_offset) / v_bus + .5f), DTC_MIN), DTC_MAX);

    /*
    sinusoidal pwm
    *dtc_u = fminf(fmaxf((u/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_v = fminf(fmaxf((v/v_bus + .5f), DTC_MIN), DTC_MAX);
    *dtc_w = fminf(fmaxf((w/v_bus + .5f), DTC_MIN), DTC_MAX);
    */
}

void linearize_dtc(float *dtc)
{
    /// linearizes the output of the inverter, which is not linear for small duty cycles ///
    float sgn = 1.0f - (2.0f * (*dtc < 0));
    if (abs(*dtc) >= .01f)
    {
        *dtc = *dtc * .986f + .014f * sgn;
    }
    else
    {
        *dtc = 2.5f * (*dtc);
    }
}

void zero_current(int *offset_1, int *offset_2)
{ // Measure zero-offset of the current sensors
    int adc1_offset = 0;
    int adc2_offset = 0;
    int n = 1024;
    for (int i = 0; i < n; i++)
    {                                         // Average n samples of the ADC
        TIM1->CCR3 = (PWM_ARR >> 1) * (1.0f); // Write duty cycles
        TIM1->CCR2 = (PWM_ARR >> 1) * (1.0f);
        TIM1->CCR1 = (PWM_ARR >> 1) * (1.0f);
        ADC1->CR2 |= 0x40000000; // Begin sample and conversion
        wait(.001);
        adc2_offset += ADC2->DR;
        adc1_offset += ADC1->DR;
    }
    *offset_1 = adc1_offset / n;
    *offset_2 = adc2_offset / n;
}

void init_controller_params(ControllerStruct *controller)
{
    controller->ki_d = KI_D;
    controller->ki_q = KI_Q;
    controller->k_d = K_SCALE * I_BW;
    controller->k_q = K_SCALE * I_BW;
    controller->alpha = 1.0f - 1.0f / (1.0f - DT * I_BW * 2.0f * PI);
}

void reset_foc(ControllerStruct *controller)
{
    TIM1->CCR3 = (PWM_ARR >> 1) * (0.5f);
    TIM1->CCR1 = (PWM_ARR >> 1) * (0.5f);
    TIM1->CCR2 = (PWM_ARR >> 1) * (0.5f);
    controller->i_d_ref = 0;
    controller->i_q_ref = 0;
    controller->i_d = 0;
    controller->i_q = 0;
    controller->i_q_filt = 0;
    controller->q_int = 0;
    controller->d_int = 0;
    controller->v_q = 0;
    controller->v_d = 0;
}

void reset_observer(ObserverStruct *observer)
{
    observer->temperature = 25.0f;
    observer->resistance = .1f;
}

void limit_current_ref(ControllerStruct *controller)
{
    float i_q_max_limit = (0.5774f * controller->v_bus - controller->dtheta_elec * WB) / R_PHASE;
    float i_q_min_limit = (-0.5774f * controller->v_bus - controller->dtheta_elec * WB) / R_PHASE;
    controller->i_q_ref = fmaxf(fminf(i_q_max_limit, controller->i_q_ref), i_q_min_limit);
}

void commutate(ControllerStruct *controller, ObserverStruct *observer, GPIOStruct *gpio, float theta)
{

    /// Update observer estimates ///
    // Resistance observer //
    // Temperature Observer //
    float t_rise = (float)observer->temperature - 25.0f;
    float q_th_in = (1.0f + .00393f * t_rise) * (controller->i_d * controller->i_d * R_PHASE * SQRT3 + controller->i_q * controller->i_q * R_PHASE * SQRT3);
    float q_th_out = t_rise * R_TH;
    observer->temperature += INV_M_TH * DT * (q_th_in - q_th_out);

    observer->resistance = (controller->v_q - SQRT3 * controller->dtheta_elec * (WB)) / controller->i_q;
    //observer->resistance = controller->v_q/controller->i_q;
    if (isnan(observer->resistance))
    {
        observer->resistance = R_PHASE;
    }
    observer->temperature2 = (double)(25.0f + ((observer->resistance * 6.0606f) - 1.0f) * 275.5f);
    double e = observer->temperature - observer->temperature2;
    observer->temperature -= .001 * e;
    //printf("%.3f\n\r", e);

    /// Commutation Loop ///
    controller->loop_count++;
    if (PHASE_ORDER)
    {                                                                                        // Check current sensor ordering
        controller->i_b = I_SCALE * (float)(controller->adc2_raw - controller->adc2_offset); // Calculate phase currents from ADC readings
        controller->i_c = I_SCALE * (float)(controller->adc1_raw - controller->adc1_offset);
    }
    else
    {
        controller->i_b = I_SCALE * (float)(controller->adc1_raw - controller->adc1_offset);
        controller->i_c = I_SCALE * (float)(controller->adc2_raw - controller->adc2_offset);
    }
    controller->i_a = -controller->i_b - controller->i_c;

    float s = FastSin(theta);
    float c = FastCos(theta);
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q); //dq0 transform on currents
                                                                                                                        //controller->i_d = 0.6666667f*(c*controller->i_a + (0.86602540378f*s-.5f*c)*controller->i_b + (-0.86602540378f*s-.5f*c)*controller->i_c);   ///Faster DQ0 Transform
                                                                                                                        //controller->i_q = 0.6666667f*(-s*controller->i_a - (-0.86602540378f*c-.5f*s)*controller->i_b - (0.86602540378f*c-.5f*s)*controller->i_c);

    controller->i_q_filt = 0.95f * controller->i_q_filt + 0.05f * controller->i_q;
    controller->i_d_filt = 0.95f * controller->i_d_filt + 0.05f * controller->i_d;

    // Filter the current references to the desired closed-loop bandwidth
    controller->i_d_ref_filt = (1.0f - controller->alpha) * controller->i_d_ref_filt + controller->alpha * controller->i_d_ref;
    controller->i_q_ref_filt = (1.0f - controller->alpha) * controller->i_q_ref_filt + controller->alpha * controller->i_q_ref;

    /// Field Weakening ///

    controller->fw_int += .001f * (0.5f * OVERMODULATION * controller->v_bus - controller->v_ref);
    controller->fw_int = fmaxf(fminf(controller->fw_int, 0.0f), -I_MAX_FW);
    controller->i_d_ref = controller->fw_int;
    //float i_cmd_mag_sq = controller->i_d_ref*controller->i_d_ref + controller->i_q_ref*controller->i_q_ref;
    limit_norm(&controller->i_d_ref, &controller->i_q_ref, I_MAX);

    /// PI Controller ///
    float i_d_error = controller->i_d_ref - controller->i_d;
    float i_q_error = controller->i_q_ref - controller->i_q; //  + cogging_current;

    // Calculate feed-forward voltages //
    float v_d_ff = SQRT3 * (1.0f * controller->i_d_ref * R_PHASE - controller->dtheta_elec * L_Q * controller->i_q); //feed-forward voltages
    float v_q_ff = SQRT3 * (1.0f * controller->i_q_ref * R_PHASE + controller->dtheta_elec * (L_D * controller->i_d + 1.0f * WB));

    // Integrate Error //
    controller->d_int += controller->k_d * controller->ki_d * i_d_error;
    controller->q_int += controller->k_q * controller->ki_q * i_q_error;

    controller->d_int = fmaxf(fminf(controller->d_int, OVERMODULATION * controller->v_bus), -OVERMODULATION * controller->v_bus);
    controller->q_int = fmaxf(fminf(controller->q_int, OVERMODULATION * controller->v_bus), -OVERMODULATION * controller->v_bus);

    //limit_norm(&controller->d_int, &controller->q_int, OVERMODULATION*controller->v_bus);
    controller->v_d = controller->k_d * i_d_error + controller->d_int; //+ v_d_ff;
    controller->v_q = controller->k_q * i_q_error + controller->q_int; //+ v_q_ff;

    controller->v_ref = sqrt(controller->v_d * controller->v_d + controller->v_q * controller->v_q);

    limit_norm(&controller->v_d, &controller->v_q, OVERMODULATION * controller->v_bus); // Normalize voltage vector to lie within curcle of radius v_bus
    float dtc_d = controller->v_d / controller->v_bus;
    float dtc_q = controller->v_q / controller->v_bus;
    linearize_dtc(&dtc_d);
    linearize_dtc(&dtc_q);
    controller->v_d = dtc_d * controller->v_bus;
    controller->v_q = dtc_q * controller->v_bus;
    abc(controller->theta_elec + 0.0f * DT * controller->dtheta_elec, controller->v_d, controller->v_q, &controller->v_u, &controller->v_v, &controller->v_w); //inverse dq0 transform on voltages
    svm(controller->v_bus, controller->v_u, controller->v_v, controller->v_w, &controller->dtc_u, &controller->dtc_v, &controller->dtc_w);                     //space vector modulation

    if (PHASE_ORDER)
    {                                                        // Check which phase order to use,
        TIM1->CCR3 = (PWM_ARR) * (1.0f - controller->dtc_u); // Write duty cycles
        TIM1->CCR2 = (PWM_ARR) * (1.0f - controller->dtc_v);
        TIM1->CCR1 = (PWM_ARR) * (1.0f - controller->dtc_w);
    }
    else
    {
        TIM1->CCR3 = (PWM_ARR) * (1.0f - controller->dtc_u);
        TIM1->CCR1 = (PWM_ARR) * (1.0f - controller->dtc_v);
        TIM1->CCR2 = (PWM_ARR) * (1.0f - controller->dtc_w);
    }

    controller->theta_elec = theta;
}

void torque_control(ControllerStruct *controller)
{
    float torque_ref = controller->kp * (controller->p_des - controller->theta_mech) + controller->t_ff + controller->kd * (controller->v_des - controller->dtheta_mech);
    //float torque_ref = -.1*(controller->p_des - controller->theta_mech);
    controller->i_q_ref = torque_ref / KT_OUT;
    controller->i_d_ref = 0.0f;
}
