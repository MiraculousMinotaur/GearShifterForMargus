#ifndef PEDALS_H
#define PEDALS_H

#include <Arduino.h>
#include <Joystick.h>
#include "Config.h"
#include "Utils.h"

#if PEDALS

// Pedal Pins
#define ACCELERATOR_PIN A1 
#define BRAKE_PIN A2
#define CLUTCH_PIN A0

// Pedal Calibration
#define ACCELERATOR_MIN_VALUE 320
#define ACCELERATOR_MAX_VALUE 900
#define BRAKE_MIN_VALUE 60
#define BRAKE_MAX_VALUE 950
#define CLUTCH_MIN_VALUE 90
#define CLUTCH_MAX_VALUE 750

void Pedals_begin();
void Pedals_update();

#endif // PEDALS

#endif // PEDALS_H
