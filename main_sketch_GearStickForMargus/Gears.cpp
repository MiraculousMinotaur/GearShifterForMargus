#include "Gears.h"
#include "Config.h"

#if GEARS

const uint8_t SixWayPins[] = {SIX_WAY_PIN_1, SIX_WAY_PIN_2, SIX_WAY_PIN_3, SIX_WAY_PIN_4, SIX_WAY_PIN_5, SIX_WAY_PIN_6};
const uint8_t ModePins[] = {MODE_PIN_1, MODE_PIN_2};
const uint8_t ImpulsePins[] = {LOWER_IMPULSE_PIN, HIGHER_IMPULSE_PIN};

// history
static uint8_t prevImpulseState = 0;
static uint8_t prevModeState = 0;
static uint8_t prevGearState = 0;

static void handleGear(bool &currentlyActive, uint8_t &prevState, const uint8_t *pins, size_t pinCount, Buttons_e firstButton)
{
  currentlyActive = false;
  for (int i = (int)pinCount - 1; i >= 0; --i)
  {
    if (!digitalRead(pins[i])) // active low
    {
      currentlyActive = true;
      uint8_t gearIndex = i + 1; // 1-based internal state
      if (prevState == 0)
      {
        // Only press when coming from neutral
        Joystick.pressButton((uint8_t)firstButton + i);
        prevState = gearIndex;
      }
      else if (prevState != gearIndex && digitalRead(pins[prevState - 1]))
      {
        // Neutral not detected during gear change: mark inactive
        currentlyActive = false;
      }
      break;
    }
  }

  if (!currentlyActive)
  {
    for (int i = (int)firstButton; i < (int)firstButton + (int)pinCount; ++i)
    {
      Joystick.releaseButton(i);
    }
    prevState = 0;
  }
}

void Gears_begin()
{
  for (size_t i = 0; i < sizeof(SixWayPins)/sizeof(SixWayPins[0]); ++i) pinMode(SixWayPins[i], INPUT_PULLUP);
  for (size_t i = 0; i < sizeof(ModePins)/sizeof(ModePins[0]); ++i) pinMode(ModePins[i], INPUT_PULLUP);
  for (size_t i = 0; i < sizeof(ImpulsePins)/sizeof(ImpulsePins[0]); ++i) pinMode(ImpulsePins[i], INPUT_PULLUP);
}

void Gears_update()
{
  bool inImpulse = false;
  bool inMode = false;
  bool inGear = false;
  handleGear(inImpulse, prevImpulseState, ImpulsePins, sizeof(ImpulsePins)/sizeof(ImpulsePins[0]), IMPULSE_1);
  handleGear(inMode, prevModeState, ModePins, sizeof(ModePins)/sizeof(ModePins[0]), REVERSE);
  handleGear(inGear, prevGearState, SixWayPins, sizeof(SixWayPins)/sizeof(SixWayPins[0]), NORMAL_1);
}

#endif // GEARS
