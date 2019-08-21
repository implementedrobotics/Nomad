#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "foc.h"
#include "mbed.h"
#include "PositionSensor.h"
#include "PreferenceWriter.h"
#include "user_config.h"

#define V_CAL 0.15f;


void order_phases(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs);
void calibrate(PositionSensor *ps, GPIOStruct *gpio, ControllerStruct *controller, PreferenceWriter *prefs);
#endif
