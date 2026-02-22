#include "Gears.h"
#include "Config.h"

#if GEARS

#include "DebugManager.h"

// MCP23017 instance (using I2C)
Adafruit_MCP23X17 mcp;

const uint8_t SixWayPins[] = {SIX_WAY_PIN_1, SIX_WAY_PIN_2, SIX_WAY_PIN_3, SIX_WAY_PIN_4, SIX_WAY_PIN_5, SIX_WAY_PIN_6};
const uint8_t ModePins[] = {MODE_PIN_1, MODE_PIN_2};
const uint8_t ImpulsePins[] = {LOWER_IMPULSE_PIN, HIGHER_IMPULSE_PIN};

// history
static uint8_t prevImpulseState = 0;
static uint8_t prevModeState = 0;
static uint8_t prevGearState = 0;

// Cached GPIO mask from last readGPIOAB() call
static uint16_t lastGPIOAB = 0xFFFF;  // Default all pins high (inactive in active-low)

// Helper function to read a single pin from the cached mask
// Note: uses active-low semantics (0 = active, 1 = inactive)
static inline bool readPinFromMask(uint16_t mask, uint8_t pinIndex)
{
  if (pinIndex < 16)
  {
    return (mask >> pinIndex) & 1;  // Returns 1 if HIGH (inactive), 0 if LOW (active)
  }
  return 1;  // Default to inactive if out of range
}

static void handleGear(bool &currentlyActive, uint8_t &prevState, const uint8_t *pins, size_t pinCount, Buttons_e firstButton, uint16_t gpioMask)
{
  currentlyActive = false;
  for (int8_t i = (int8_t)pinCount - 1; i >= 0; --i)
  {
    if (!readPinFromMask(gpioMask, pins[i]))  // active low (bit = 0 means active)
    {
      currentlyActive = true;
      uint8_t gearIndex = i + 1;  // 1-based internal state
      if (prevState == 0)
      {
        // Only press when coming from neutral
        Joystick.pressButton((uint8_t)firstButton + i);
        prevState = gearIndex;
      }
      else if (prevState != gearIndex && readPinFromMask(gpioMask, pins[prevState - 1]))
      {
        // Neutral not detected during gear change: mark inactive
        currentlyActive = false;
      }
      break;
    }
  }

  if (!currentlyActive)
  {
    for (int16_t i = (int16_t)firstButton; i < (int16_t)firstButton + (int16_t)pinCount; ++i)
    {
      Joystick.releaseButton((uint8_t)i);
    }
    prevState = 0;
  }
}

void Gears_begin()
{
  // MCP23017 is already initialized in main_sketch setup via I2C power enable and Wire.begin()
  // This function is called after device initialization, so mcp.begin() has already been called.
  //set all MCP pins as inputs with pull-ups (active-low logic for gear switches)
  for (size_t i = 0; i < 16; ++i) 
  {
    mcp.pinMode(i, INPUT_PULLUP);  // Enable internal pull-up
  }
  // Configure all used MCP pins as inputs with internal pull-ups
  /*for (size_t i = 0; i < sizeof(SixWayPins)/sizeof(SixWayPins[0]); ++i) 
  {
    mcp.pinMode(SixWayPins[i], INPUT_PULLUP);  // Enable internal pull-up
  }
  for (size_t i = 0; i < sizeof(ModePins)/sizeof(ModePins[0]); ++i)
  {
    mcp.pinMode(ModePins[i], INPUT_PULLUP);
  }
  for (size_t i = 0; i < sizeof(ImpulsePins)/sizeof(ImpulsePins[0]); ++i)
  {
    mcp.pinMode(ImpulsePins[i], INPUT_PULLUP);
  }*/
  
  // Read initial state
  lastGPIOAB = mcp.readGPIOAB();
}

void Gears_update()
{
  // Read all 16 MCP23017 pins in a single I2C transaction for efficiency
  lastGPIOAB = mcp.readGPIOAB();
  
  bool inImpulse = false;
  bool inMode = false;
  bool inGear = false;
  
  handleGear(inImpulse, prevImpulseState, ImpulsePins, sizeof(ImpulsePins)/sizeof(ImpulsePins[0]), IMPULSE_1, lastGPIOAB);
  handleGear(inMode, prevModeState, ModePins, sizeof(ModePins)/sizeof(ModePins[0]), REVERSE, lastGPIOAB);
  handleGear(inGear, prevGearState, SixWayPins, sizeof(SixWayPins)/sizeof(SixWayPins[0]), NORMAL_1, lastGPIOAB);
}

// ===== Debug Report Function =====
#if DEBUG
void Gears_reportDebug(struct DebugTelemetry_t *tel)
{
  if (tel) {
    tel->gears_gpio = lastGPIOAB;
  }
}
#endif

#endif // GEARS
