#ifndef PEDALS_H
#define PEDALS_H

#include <Arduino.h>
#include <Joystick.h>
#include "Config.h"
#include "Utils.h"

#if PEDALS

#include <Adafruit_ADS1X15.h>

// Pedal Pins: using external ADS1115 channels; MCU analog fallback removed
// (ADS1115 channels used: ADS_CH_ACCEL/1/2 defined in Config.h)

// Pedal Calibration TODO: Recalibrate for ADS1115
const int16_t ACCELERATOR_MIN_VALUE = 320;
const int16_t ACCELERATOR_MAX_VALUE = 900;
const int16_t BRAKE_MIN_VALUE = 60;
const int16_t BRAKE_MAX_VALUE = 950;
const int16_t CLUTCH_MIN_VALUE = 90;
const int16_t CLUTCH_MAX_VALUE = 750;
const int16_t PEDALS_REFERENCE_DEFAULT = 13333;  // approximately 2.5v

// Extern ADS1115 instance (defined in Pedals.cpp)
extern Adafruit_ADS1115 ads;

void Pedals_begin();
void Pedals_update();

#endif // PEDALS

#endif // PEDALS_H
