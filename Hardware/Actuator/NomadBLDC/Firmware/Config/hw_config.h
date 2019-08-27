#ifndef HW_CONFIG_H
#define HW_CONFIG_H

#define PIN_U PA_10
#define PIN_V PA_9
#define PIN_W PA_8
#define ENABLE_PIN PA_11        // Enable gate drive pin
#define LED_PIN         PC_5        // LED Pin
#define I_SCALE 0.02014160156f  // Amps per A/D Count
#define V_SCALE 0.012890625f     // Bus volts per A/D Count
#define DTC_MAX 0.94f          // Max phase duty cycle
#define DTC_MIN 0.0f          // Min phase duty cycle
#define PWM_ARR 0x8CA           /// timer autoreload value

#endif
