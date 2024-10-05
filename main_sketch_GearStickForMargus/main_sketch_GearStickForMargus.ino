#include <Joystick.h>

// Six way pins
#define SIX_WAY_PIN_1 5
#define SIX_WAY_PIN_2 A5
#define SIX_WAY_PIN_3 6
#define SIX_WAY_PIN_4 A4
#define SIX_WAY_PIN_5 7
#define SIX_WAY_PIN_6 A3

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


const uint8_t SixWayPins[] = {SIX_WAY_PIN_1, SIX_WAY_PIN_2, SIX_WAY_PIN_3, SIX_WAY_PIN_4, SIX_WAY_PIN_5, SIX_WAY_PIN_6};
const uint8_t ModePins[] = {MODE_PIN_1, MODE_PIN_2};
const uint8_t ImpulsePins[] = {LOWER_IMPULSE_PIN, HIGHER_IMPULSE_PIN};

// history
uint8_t prevImpulseState = 0;
uint8_t prevModeState = 0;
uint8_t prevGearState = 0;

Joystick_ Joystick;

void setup() {
  // Prepare InputPins
  for(int8_t i = sizeof(SixWayPins); i > -1; i--) {pinMode(SixWayPins[i],INPUT_PULLUP);}
  for(int8_t i = sizeof(ModePins); i > -1; i--)   {pinMode(ModePins[i],INPUT_PULLUP);}
  for(int8_t i = sizeof(ImpulsePins); i > -1; i--){pinMode(ImpulsePins[i],INPUT_PULLUP);}

  // Initialize Joystick Library
  Joystick.begin();
}

void loop() 
{
  
  bool inGear = false;
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
  delay(50);
}
