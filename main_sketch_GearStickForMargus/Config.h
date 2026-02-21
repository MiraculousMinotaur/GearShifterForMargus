#ifndef CONFIG_H
#define CONFIG_H

// Feature flags -- centralized so .cpp/.h files can see the same values as the .ino
#define DEBUG 1
#define PEDALS 1
#define GEARS 1
#define WHEEL 1
#if WHEEL
#define FFB 1 // FFB currently only effects steering
#else
#define FFB 0
#endif

// Default pin assignments (can be overridden per-board)
// TODO: Verify these pins don't conflict with GEARS/WHEEL. If conflict exists, provide alternatives in Config.h or a board-specific header.
// NOTE: ACS712_PIN_SENSE/POWER are A5/A4 (typical I2C pins on AVR/32u4). If enabling PEDALS/GEARS with I2C, consider moving ACS712 to non-I2C pins.
#define ACS712_PIN_SENSE A5
#define ACS712_PIN_POWER A4

// I2C Configuration for ADS1115 (pedals) and MCP23017 (gears)
// ADS1115 -- Continuous conversion mode for pedal potentiometers
#define ADS1115_I2C_ADDR 0x48
#define ADS1115_GAIN GAIN_TWOTHIRDS          // Adafruit_ADS1X15 constant; use GAIN_TWOTHIRDS for 0-6.144V, GAIN_ONE for 0-4.096V
#define ADS1115_DATARATE RATE_ADS1115_250SPS // SPS (samples per second): 8, 16, 32, 64, 128, 250, 475, 860
#define ADS_CH_ACCEL 1                       // ADS1115 channel 1 (A1) -> Accelerator
#define ADS_CH_BRAKE 0                       // ADS1115 channel 0 (A0) -> Brake
#define ADS_CH_CLUTCH 2                      // ADS1115 channel 2 (A2) -> Clutch
// MCP23017 -- GPIO Expander for gear inputs (10 inputs total: 6 six-way + 2 mode + 2 impulse)
#define MCP23017_I2C_ADDR 0x20          // Default MCP23017 address (all ADDR pins low)
#define MCP_GEAR_SIXWAY_FIRST_PIN 2     // MCP pins 0-5 for six-way switches (GPA0-GPA5)
#define MCP_GEAR_MODE_FIRST_PIN 8       // MCP pins 6-7 for mode switches (GPA6-GPA7)
#define MCP_GEAR_IMPULSE_FIRST_PIN 14   // MCP pins 8-9 for impulse switches (GPB0-GPB1)

// I2C Power Control
#define I2C_POWER_ENABLE_PIN 8         // Digital pin (D8) to enable external I2C device power supply
#define I2C_POWER_ENABLE_ACTIVE_HIGH 1 // Set to 1 if driving pin HIGH enables power; set 0 for active-low
#define I2C_POWER_STABILIZE_MS 50      // Delay (ms) after enabling I2C power before initializing devices

#include <Joystick.h>
// Make the global Joystick object accessible from .cpp modules
extern Joystick_ Joystick;

// Endpoint enforcement tuning (used by Wheel.cpp)
// Higher values make the endpoint enforcement engage sooner (smaller breach)
// Set to 24 so that ~1/8 of a full rotation (~300 encoder clicks) will reach MAX_PWM
#define ENDPOINT_BAND 24

#endif // CONFIG_H
