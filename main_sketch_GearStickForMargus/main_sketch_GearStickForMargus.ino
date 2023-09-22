#include <Joystick.h>

// Six way pins
#define SIX_WAY_PIN_1 1
#define SIX_WAY_PIN_2 2
#define SIX_WAY_PIN_3 3
#define SIX_WAY_PIN_4 4
#define SIX_WAY_PIN_5 5
#define SIX_WAY_PIN_6 6

// Mode switch pins
#define MODE_PIN_1 7 //Reverse
#define MODE_PIN_2 8 //Low_Range

// Impulse switch pins also works as High/Low for normal mode 
#define LOWER_IMPULSE_PIN   9
#define HIGHER_IMPULSE_PIN 10

#define MAIN_STICK_SIZE 6

enum Buttons{
  NEUTRAL_0 = 0,
  NORMAL_LOW_1,
  NORMAL_LOW_2,
  NORMAL_LOW_3,
  NORMAL_LOW_4,
  NORMAL_LOW_5,
  NORMAL_LOW_6,
  NORMAL_HIGH_7,
  NORMAL_HIGH_8,
  NORMAL_HIGH_9,
  NORMAL_HIGH_10,
  NORMAL_HIGH_11,
  NORMAL_HIGH_12,
  REVERSE_13,
  REVERSE_14,
  REVERSE_15,
  REVERSE_16,
  REVERSE_17,
  REVERSE_18,
  LOW_RANGE_19,
  LOW_RANGE_20,
  LOW_RANGE_21,
  LOW_RANGE_22,
  LOW_RANGE_23,
  LOW_RANGE_24,
  LOWER_IMPULSE_25,
  HIGHER_IMPULSE_26
};

enum Modes{
  NORMAL = 0,
  REVERSE,
  LOW_RANGE_MODE
};

enum Ranges{
  LOW_RANGE = 0,
  HIGH_RANGE
};


const uint8_t SixWayPins[] = {SIX_WAY_PIN_1, SIX_WAY_PIN_2, SIX_WAY_PIN_3, SIX_WAY_PIN_4, SIX_WAY_PIN_5, SIX_WAY_PIN_6};
const uint8_t ModePins[] = {MODE_PIN_1, MODE_PIN_2};
const uint8_t ImpulsePins[] = {LOWER_IMPULSE_PIN, HIGHER_IMPULSE_PIN};

// history
Ranges range = LOW;
uint8_t prevImpulseState = 0;
Buttons prevGear = NEUTRAL_0;

Joystick_ Joystick;

void setup() {
  // Prepare InputPins
  for(int8_t i = sizeof(SixWayPins); i > -1; i--) {pinMode(SixWayPins[i],INPUT_PULLUP);}
  for(int8_t i = sizeof(ModePins); i > -1; i--)   {pinMode(ModePins[i],INPUT_PULLUP);}
  for(int8_t i = sizeof(ImpulsePins); i > -1; i--){pinMode(ImpulsePins[i],INPUT_PULLUP);}

  // Initialize Joystick Library
  Joystick.begin();
  Joystick.pressButton(NEUTRAL_0);
}

void loop() 
{
  
  bool inGear = false;
  bool inMode = false;
  bool inImpulse = false;
  Buttons gear = NEUTRAL_0;
  Modes mode = NORMAL;

  // Impulse and Low/high Selection
  for(int8_t i = sizeof(ImpulsePins); i > 0; i--)
  {
    if(!digitalRead(ImpulsePins[i-1]))
    {
      inImpulse = true;
      range = i-1;
      if(i != prevImpulseState)
      {
        Joystick.pressButton(LOWER_IMPULSE_25 + range);
        prevImpulseState = i;
      }
    }
  }
  if(!inImpulse)
  {
    Joystick.releaseButton(LOWER_IMPULSE_25);
    Joystick.releaseButton(HIGHER_IMPULSE_26);
  }

  //SixWay and mode selection 
  for(int8_t i = sizeof(SixWayPins); i > 0; i--) 
  {
    if(!digitalRead(SixWayPins[i-1]))
    {
      inGear = true;
      gear = i;
    }
  }
  if(inGear)
  {
      for(int8_t i = sizeof(ModePins); i > 0; i--)   
      {
        if(!digitalRead(ModePins[i-1]))
        {
          inMode = true;
          mode = i;
        }
      }
      switch(mode)
      {
        case NORMAL:
          if(HIGH_RANGE == range){gear = gear + MAIN_STICK_SIZE;}
          break;
        case REVERSE:
          gear = gear + MAIN_STICK_SIZE * 2;
          break;
        case LOW_RANGE_MODE:
          gear = gear + MAIN_STICK_SIZE * 3;
          break;
        default:break;
        //This should not happen
      }

  }
  else{gear = 0;}
  if(gear != prevGear)
  {
    for(int8_t i = LOW_RANGE_24;i > NEUTRAL_0-1;i--){Joystick.releaseButton(i);}
    Joystick.pressButton(gear);
    prevGear = gear;
  }


  delay(50);
}
