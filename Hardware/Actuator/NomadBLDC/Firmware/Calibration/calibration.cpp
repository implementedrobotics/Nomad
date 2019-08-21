/// Calibration procedures for determining position sensor offset, 
/// phase ordering, and position sensor linearization
/// 

#include "calibration.h"
#include "foc.h"
#include "PreferenceWriter.h"
#include "user_config.h"
#include "motor_config.h"
#include "current_controller_config.h"

void order_phases(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs){   
    
    ///Checks phase order, to ensure that positive Q current produces
    ///torque in the positive direction wrt the position sensor.
    printf("\n\r Checking phase ordering\n\r");
    float theta_ref = 0;
    float theta_actual = 0;
    float v_d = V_CAL;                                                             //Put all volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0;
    float dtc_u, dtc_v, dtc_w = .5f;
    int sample_counter = 0;
    
    ///Set voltage angle to zero, wait for rotor position to settle
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 //inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                            //space vector modulation
    for(int i = 0; i<20000; i++){
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        // Set duty cycles
        TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
        TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
        wait_us(100);
        }
    //ps->ZeroPosition();
    ps->Sample(DT); 
    wait_us(1000);
    //float theta_start = ps->GetMechPositionFixed();                                  //get initial rotor position
    float theta_start;
    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //Calculate phase currents from ADC readings
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
    controller->i_a = -controller->i_b - controller->i_c;
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));
    printf("\n\rCurrent\n\r");
    printf("%f    %f   %f\n\r\n\r", controller->i_d, controller->i_q, current);
    /// Rotate voltage angle
    while(theta_ref < 4*PI){                                                    //rotate for 2 electrical cycles
        abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                             //inverse dq0 transform on voltages
        svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                        //space vector modulation
        wait_us(100);
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        //Set duty cycles
        TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
        TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
       ps->Sample(DT);                                                            //sample position sensor
       theta_actual = ps->GetMechPositionFixed();
       if(theta_ref==0){theta_start = theta_actual;}
       if(sample_counter > 200){
           sample_counter = 0 ;
        printf("%.4f   %.4f\n\r", theta_ref/(NPP), theta_actual);
        }
        sample_counter++;
       theta_ref += 0.001f;
        }
    float theta_end = ps->GetMechPositionFixed();
    int direction = (theta_end - theta_start)>0;
    printf("Theta Start:   %f    Theta End:  %f\n\r", theta_start, theta_end);
    printf("Direction:  %d\n\r", direction);
    if(direction){printf("Phasing correct\n\r");}
    else if(!direction){printf("Phasing incorrect.  Swapping phases V and W\n\r");}
    PHASE_ORDER = direction;
    }
    
    
void calibrate(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs){
    /// Measures the electrical angle offset of the position sensor
    /// and (in the future) corrects nonlinearity due to position sensor eccentricity
    printf("Starting calibration procedure\n\r");
    float * error_f;
    float * error_b;
    int * lut;
    int * raw_f;
    int * raw_b;
    float * error;
    float * error_filt;
    
    const int n = 128*NPP;                                                      // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
    const int n2 = 40;                                                          // increments between saved samples (for smoothing motion)
    float delta = 2*PI*NPP/(n*n2);                                              // change in angle between samples
    error_f = new float[n]();                                                     // error vector rotating forwards
    error_b = new float[n]();                                                     // error vector rotating backwards
    const int  n_lut = 128;
    lut = new int[n_lut]();                                                        // clear any old lookup table before starting.
    
    error = new float[n]();
    const int window = 128;
    error_filt = new float[n]();
    float cogging_current[window] = {0};
    
    ps->WriteLUT(lut); 
    raw_f = new int[n]();
    raw_b = new int[n]();
    float theta_ref = 0;
    float theta_actual = 0;
    float v_d = V_CAL;                                                             // Put volts on the D-Axis
    float v_q = 0.0f;
    float v_u, v_v, v_w = 0;
    float dtc_u, dtc_v, dtc_w = .5f;
    
        
    ///Set voltage angle to zero, wait for rotor position to settle
    abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                                 // inverse dq0 transform on voltages
    svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                            // space vector modulation
    for(int i = 0; i<40000; i++){
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);                                        // Set duty cycles
        if(PHASE_ORDER){                                   
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
        else{
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
        wait_us(100);
        }
    ps->Sample(DT);   
    controller->i_b = I_SCALE*(float)(controller->adc2_raw - controller->adc2_offset);    //Calculate phase currents from ADC readings
    controller->i_c = I_SCALE*(float)(controller->adc1_raw - controller->adc1_offset);
    controller->i_a = -controller->i_b - controller->i_c;
    dq0(controller->theta_elec, controller->i_a, controller->i_b, controller->i_c, &controller->i_d, &controller->i_q);    //dq0 transform on currents
    float current = sqrt(pow(controller->i_d, 2) + pow(controller->i_q, 2));
    printf(" Current Angle : Rotor Angle : Raw Encoder \n\r\n\r");
    for(int i = 0; i<n; i++){                                                   // rotate forwards
       for(int j = 0; j<n2; j++){   
        theta_ref += delta;
       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                              // inverse dq0 transform on voltages
       svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                         // space vector modulation
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);
        if(PHASE_ORDER){                                                        // Check phase ordering
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);                                    // Set duty cycles
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
        else{
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
            wait_us(100);
            ps->Sample(DT);
        }
       ps->Sample(DT);
       theta_actual = ps->GetMechPositionFixed();
       error_f[i] = theta_ref/NPP - theta_actual;
       raw_f[i] = ps->GetRawPosition();
        printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_f[i]);
       //theta_ref += delta;
        }
    
    for(int i = 0; i<n; i++){                                                   // rotate backwards
       for(int j = 0; j<n2; j++){
       theta_ref -= delta;
       abc(theta_ref, v_d, v_q, &v_u, &v_v, &v_w);                              // inverse dq0 transform on voltages
       svm(1.0, v_u, v_v, v_w, &dtc_u, &dtc_v, &dtc_w);                         // space vector modulation
        TIM1->CCR3 = (PWM_ARR>>1)*(1.0f-dtc_u);
        if(PHASE_ORDER){
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
        else{
            TIM1->CCR1 = (PWM_ARR>>1)*(1.0f-dtc_v);
            TIM1->CCR2 = (PWM_ARR>>1)*(1.0f-dtc_w);
            }
            wait_us(100);
            ps->Sample(DT);
        }
       ps->Sample(DT);                                                            // sample position sensor
       theta_actual = ps->GetMechPositionFixed();                                    // get mechanical position
       error_b[i] = theta_ref/NPP - theta_actual;
       raw_b[i] = ps->GetRawPosition();
       printf("%.4f   %.4f    %d\n\r", theta_ref/(NPP), theta_actual, raw_b[i]);
       //theta_ref -= delta;
        }    
        
        float offset = 0;                                  
        for(int i = 0; i<n; i++){
            offset += (error_f[i] + error_b[n-1-i])/(2.0f*n);                   // calclate average position sensor offset
            }
        offset = fmod(offset*NPP, 2*PI);                                        // convert mechanical angle to electrical angle
        
            
        ps->SetElecOffset(offset);                                              // Set position sensor offset
        __float_reg[0] = offset;
        E_OFFSET = offset;
        
        /// Perform filtering to linearize position sensor eccentricity
        /// FIR n-sample average, where n = number of samples in one electrical cycle
        /// This filter has zero gain at electrical frequency and all integer multiples
        /// So cogging effects should be completely filtered out.
        
        
        float mean = 0;
        for (int i = 0; i<n; i++){                                              //Average the forward and back directions
            error[i] = 0.5f*(error_f[i] + error_b[n-i-1]);
            }
        for (int i = 0; i<n; i++){
            for(int j = 0; j<window; j++){
                int ind = -window/2 + j + i;                                    // Indexes from -window/2 to + window/2
                if(ind<0){
                    ind += n;}                                                  // Moving average wraps around
                else if(ind > n-1) {
                    ind -= n;}
                error_filt[i] += error[ind]/(float)window;
                }
            if(i<window){
                cogging_current[i] = current*sinf((error[i] - error_filt[i])*NPP);
                }
            //printf("%.4f   %4f    %.4f   %.4f\n\r", error[i], error_filt[i], error_f[i], error_b[i]);
            mean += error_filt[i]/n;
            }
        int raw_offset = (raw_f[0] + raw_b[n-1])/2;                             //Insensitive to errors in this direction, so 2 points is plenty
        
        
        printf("\n\r Encoder non-linearity compensation table\n\r");
        printf(" Sample Number : Lookup Index : Lookup Value\n\r\n\r");
        for (int i = 0; i<n_lut; i++){                                          // build lookup table
            int ind = (raw_offset>>7) + i;
            if(ind > (n_lut-1)){ 
                ind -= n_lut;
                }
            lut[ind] = (int) ((error_filt[i*NPP] - mean)*(float)(ps->GetCPR())/(2.0f*PI));
            printf("%d   %d   %d \n\r", i, ind, lut[ind]);
            wait(.001);
            }
            
        ps->WriteLUT(lut);                                                      // write lookup table to position sensor object
        //memcpy(controller->cogging, cogging_current, sizeof(controller->cogging));  //compensation doesn't actually work yet....
        memcpy(&ENCODER_LUT, lut, sizeof(lut));                                 // copy the lookup table to the flash array
        printf("\n\rEncoder Electrical Offset (rad) %f\n\r",  offset);
        
        if (!prefs->ready()) prefs->open();
        prefs->flush();                                                         // write offset and lookup table to flash
        prefs->close();
        
        delete[] error_f;       //gotta free up that ram
        delete[] error_b;
        delete[] lut;
        delete[] raw_f;
        delete[] raw_b;

    }