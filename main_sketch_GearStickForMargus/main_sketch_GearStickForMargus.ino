#include <Joystick.h>

#define DEBUG 0

// Pedal Pins

#define ACCELERATOR_PIN A0
#define BRAKE_PIN A1
#define CLUTCH_PIN A2

// Pedal Range
#define PEDAL_MIN_VALUE 0 
#define PEDAL_MAX_VALUE 512
#define PEDAL_RANGE (PEDAL_MAX_VALUE-PEDAL_MIN_VALUE)

// Pedal Calibration
#define ACCELERATOR_REAL_MIN_VALUE 375 
#define ACCELERATOR_REAL_MAX_VALUE 775
#define ACCELERATOR_REAL_RANGE (ACCELERATOR_REAL_MAX_VALUE-ACCELERATOR_REAL_MIN_VALUE)
#define ACCELERATOR_REAL_TO_OUT_CONVERSION (ACCELERATOR_REAL_RANGE/PEDAL_RANGE)
#define BRAKE_REAL_MIN_VALUE 90 
#define BRAKE_REAL_MAX_VALUE 700
#define BRAKE_REAL_RANGE (BRAKE_REAL_MAX_VALUE-BRAKE_REAL_MIN_VALUE)
#define BRAKE_REAL_TO_OUT_CONVERSION (BRAKE_REAL_RANGE/PEDAL_RANGE)
#define CLUTCH_REAL_MIN_VALUE 120 
#define CLUTCH_REAL_MAX_VALUE 700
#define CLUTCH_REAL_RANGE (CLUTCH_REAL_MAX_VALUE-CLUTCH_REAL_MIN_VALUE)
#define CLUTCH_REAL_TO_OUT_CONVERSION (CLUTCH_REAL_RANGE/PEDAL_RANGE)

// Six way pins
#define SIX_WAY_PIN_1 0
#define SIX_WAY_PIN_2 1
#define SIX_WAY_PIN_3 2
#define SIX_WAY_PIN_4 3
#define SIX_WAY_PIN_5 4
#define SIX_WAY_PIN_6 5

// Mode switch pins
#define MODE_PIN_1 10 //Reverse
#define MODE_PIN_2 11 //Low_Range

// Impulse switch pins also works as High/Low for normal mode 
#define LOWER_IMPULSE_PIN   9
#define HIGHER_IMPULSE_PIN 8

#define MAIN_STICK_SIZE 6

enum Buttons{
  NORMAL_1 = 0,
  NORMAL_2,
  NORMAL_3,
  NORMAL_4,
  NORMAL_5,
  NORMAL_6,
  REVERSE,
  HIGH_RANGE,
  IMPULSE_1,
  IMPULSE_2
};

int32_t LimitPedal(int value, int32_t min, int32_t max)
{
  if(value < min){return min;}
  if(value > max){return max;}
  return (int32_t)value;
}


const uint8_t SixWayPins[] = {SIX_WAY_PIN_1, SIX_WAY_PIN_2, SIX_WAY_PIN_3, SIX_WAY_PIN_4, SIX_WAY_PIN_5, SIX_WAY_PIN_6};
const uint8_t ModePins[] = {MODE_PIN_1, MODE_PIN_2};
const uint8_t ImpulsePins[] = {LOWER_IMPULSE_PIN, HIGHER_IMPULSE_PIN};

// history
uint8_t prevImpulseState = 0;
uint8_t prevModeState = 0;
uint8_t prevGearState = 0;

Joystick_ Joystick;

void setup() {
#if DEBUG
  Serial.begin(9600);
#endif
  // Initalize pedals
  Joystick.setXAxisRange(PEDAL_MIN_VALUE, PEDAL_MAX_VALUE);
  Joystick.setYAxisRange(PEDAL_MIN_VALUE, PEDAL_MAX_VALUE);
  Joystick.setZAxisRange(PEDAL_MIN_VALUE, PEDAL_MAX_VALUE);

  // Prepare InputPins
  for(int8_t i = sizeof(SixWayPins); i > -1; i--) {pinMode(SixWayPins[i],INPUT_PULLUP);}
  for(int8_t i = sizeof(ModePins); i > -1; i--)   {pinMode(ModePins[i],INPUT_PULLUP);}
  for(int8_t i = sizeof(ImpulsePins); i > -1; i--){pinMode(ImpulsePins[i],INPUT_PULLUP);}

  // Initialize Joystick Library
  Joystick.begin();
}

void loop() 
{
  
  /*bool inGear = false;
  bool inMode = false;
  bool inImpulse = false;

  // Impulse and Low/high Selection
  for(int8_t i = sizeof(ImpulsePins); i > 0; i--)
  {
    if(!digitalRead(ImpulsePins[i-1]))
    {
      inImpulse = true;
      if(!prevImpulseState) // gear change can only happen through neutral
      {
        Joystick.pressButton(IMPULSE_1 + i-1);
        prevImpulseState = i;
      }
      else if(prevImpulseState != i && digitalRead(ImpulsePins[prevImpulseState-1])){inImpulse = false;} // Neutral not detected during gear change
      break;//only read one of the gears
    }
  }
  if(!inImpulse)
  {
    Joystick.releaseButton(IMPULSE_1);
    Joystick.releaseButton(IMPULSE_2);
    prevImpulseState = 0;
  }

  // mode Latch Selection
  for(int8_t i = sizeof(ModePins); i > 0; i--)
  {
    if(!digitalRead(ModePins[i-1]))
    {
      inMode = true;
      if(!prevModeState) // gear change can only happen through neutral
      {
        Joystick.pressButton(REVERSE + i-1);
        prevModeState = i;
      }
      else if(prevModeState != i && digitalRead(ModePins[prevModeState-1])){inMode = false;} // Neutral not detected during gear change
      break;//only read one of the gears
    }
  }
  if(!inMode)
  {
    Joystick.releaseButton(REVERSE);
    Joystick.releaseButton(HIGH_RANGE);
    prevModeState = 0;
  }

  //SixWay
  for(int8_t i = sizeof(SixWayPins); i > 0; i--) 
  {
    if(!digitalRead(SixWayPins[i-1]))
    {
      inGear = true;
      if(!prevGearState) // gear change can only happen through neutral
      {
        Joystick.pressButton(i-1);
        prevGearState = i;
      }
      else if(prevGearState != i && digitalRead(SixWayPins[prevGearState-1])){inGear = false;} // Neutral not detected during gear change
      break;//only read one of the gears
    }
  }
  if(!inGear)
  {
    for(int8_t i = NORMAL_6;i > NORMAL_1-1;i--){Joystick.releaseButton(i);}
    prevGearState = 0;
  }
*/
  //Pedal Handeling
  int32_t pedal = 0;
  pedal = analogRead(ACCELERATOR_PIN);
#if DEBUG
  Serial.print("Axelerator: ");
  Serial.print(pedal);
#endif
  pedal = pedal - ACCELERATOR_REAL_MIN_VALUE;
  pedal = (ACCELERATOR_REAL_TO_OUT_CONVERSION > 0) ? pedal/ACCELERATOR_REAL_TO_OUT_CONVERSION : pedal;
  pedal = LimitPedal(pedal, PEDAL_MIN_VALUE, PEDAL_MAX_VALUE);
  Joystick.setXAxis(pedal);
#if DEBUG
  Serial.print(" ");
  Serial.print(pedal);
#endif
  

  pedal = 0;
  pedal = analogRead(BRAKE_PIN);
#if DEBUG
  Serial.print(" Break: ");
  Serial.print(pedal);
#endif
  pedal = pedal - BRAKE_REAL_MIN_VALUE;
  pedal = (BRAKE_REAL_TO_OUT_CONVERSION > 0) ? pedal/BRAKE_REAL_TO_OUT_CONVERSION : pedal;
  pedal = LimitPedal(pedal, PEDAL_MIN_VALUE, PEDAL_MAX_VALUE);
  Joystick.setYAxis(pedal);
#if DEBUG
  Serial.print(" ");
  Serial.print(pedal);
#endif

  pedal = 0;
  pedal = analogRead(CLUTCH_PIN);
#if DEBUG
  Serial.print(" Clutch: ");
  Serial.print(pedal);
#endif
  pedal = pedal - CLUTCH_REAL_MIN_VALUE;
  pedal = (CLUTCH_REAL_TO_OUT_CONVERSION > 0) ? pedal/CLUTCH_REAL_TO_OUT_CONVERSION : pedal;
  pedal = LimitPedal(pedal, PEDAL_MIN_VALUE, PEDAL_MAX_VALUE);
  Joystick.setZAxis(pedal);
#if DEBUG
  Serial.print(" ");
  Serial.println(pedal);
#endif

  delay(50);
}
