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
  int pedal = analogRead(ACCELERATOR_PIN);
  pedal = limitVal(pedal, ACCELERATOR_MIN_VALUE, ACCELERATOR_MAX_VALUE);
  Joystick.setAccelerator(pedal);

  pedal = analogRead(BRAKE_PIN);
  pedal = limitVal(pedal, BRAKE_MIN_VALUE, BRAKE_MAX_VALUE);
  Joystick.setBrake(pedal);

  pedal = analogRead(CLUTCH_PIN);
  pedal = limitVal(pedal, CLUTCH_MIN_VALUE, CLUTCH_MAX_VALUE);
  Joystick.setZAxis(pedal);
}

#endif // PEDALS
