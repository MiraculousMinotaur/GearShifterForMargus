#include "Pedals.h"

#if PEDALS

void Pedals_begin()
{
  Joystick.setAcceleratorRange(ACCELERATOR_MIN_VALUE, ACCELERATOR_MAX_VALUE);
  Joystick.setBrakeRange(BRAKE_MIN_VALUE, BRAKE_MAX_VALUE);
  Joystick.setZAxisRange(CLUTCH_MIN_VALUE, CLUTCH_MAX_VALUE);
}

void Pedals_update()
{
  // Pedals analog reads commented out — pedal inputs will be moved to external ADC over I2C.
  // Keep logic here for future use; currently no ADC reads to avoid conflicts with free-running ADC.
  /*
  int pedal = analogRead(ACCELERATOR_PIN);
  pedal = limitVal(pedal, ACCELERATOR_MIN_VALUE, ACCELERATOR_MAX_VALUE);
  Joystick.setAccelerator(pedal);

  pedal = analogRead(BRAKE_PIN);
  pedal = limitVal(pedal, BRAKE_MIN_VALUE, BRAKE_MAX_VALUE);
  Joystick.setBrake(pedal);

  pedal = analogRead(CLUTCH_PIN);
  pedal = limitVal(pedal, CLUTCH_MIN_VALUE, CLUTCH_MAX_VALUE);
  Joystick.setZAxis(pedal);
  */
}

#endif // PEDALS
