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
const int16_t ACCELERATOR_MIN_VALUE = 10000;
const int16_t ACCELERATOR_MAX_VALUE = 24000;
const int16_t BRAKE_MIN_VALUE = 2200;
const int16_t BRAKE_MAX_VALUE = 20000;
const int16_t CLUTCH_MIN_VALUE = 6000;
const int16_t CLUTCH_MAX_VALUE = 16000;
// Extern ADS1115 instance (defined in Pedals.cpp)
extern Adafruit_ADS1115 ads;

enum ads_state_t:uint8_t
{
  IDLE = 0,
  WAITING_ON_CONVERSION = 1,
  CONVERTED = 2
};

ads_state_t get_Ads_State(void);

void Pedals_begin(void);
void Pedals_update(void);

// Debug report function (opaque pointer, actual type defined in DebugManager.h)
#if DEBUG
struct DebugTelemetry_t;
void Pedals_reportDebug(struct DebugTelemetry_t *tel);
#endif

#endif // PEDALS

#endif // PEDALS_H
