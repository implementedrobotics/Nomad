#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define PIN_U PA_10
#define PIN_V PA_9
#define PIN_W PA_8
#define ENABLE_PIN PA_11        // Enable gate drive pin
#define LED         PC_5        // LED Pin
#define I_SCALE 0.02014160156f  // Amps per A/D Count
#define V_SCALE 0.012890625f     // Bus volts per A/D Count
#define DTC_MAX 0.94f          // Max phase duty cycle
#define DTC_MIN 0.0f          // Min phase duty cycle
#define PWM_ARR 0x8CA           /// timer autoreload value

static float inverter_tab[16] = {2.5f, 2.4f, 2.3f, 2.2f, 2.1f, 2.0f, 1.9f, 1.8f, 1.7f, 1.6f, 1.59f, 1.58f, 1.57f, 1.56f, 1.55f, 1.5f};


#endif
