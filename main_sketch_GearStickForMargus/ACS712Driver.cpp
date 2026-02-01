#include "ACS712Driver.h"
#include "Wheel.h"
#include "Config.h"
#include "Utils.h"
#include "motor.h"

// Configurable parameters (raw ADC units, no conversion)
static const float DEFAULT_KP = 1.0f;
static const float DEFAULT_KI = 0.1f;
static const float DEFAULT_KD = 0.0f;
static const int MAX_ADC_DELTA = 130;      // max ADC delta from zero (corresponds to ~10A)
static const int SHUTOFF_ADC_DELTA = 182;  // emergency shutoff threshold (corresponds to ~14A)

// ADC calibration (10-bit ADC value at zero current)
static int zeroADC = 513;

// Sampling & timing // TODO sampling tuning will be handled by register HW config handled in SetupADC() function removed unneccesary values
static const uint16_t CONTROL_HZ = 240; // default control rate (Hz)
static const uint32_t CONTROL_INTERVAL_US = 1000000UL / CONTROL_HZ;
static const uint8_t SAMPLES_PER_CYCLE = 4;
static const uint32_t SAMPLE_INTERVAL_US = CONTROL_INTERVAL_US / SAMPLES_PER_CYCLE; // approx spacing

// Motor limits tied to setMotor scaling
static const int MAX_SETMOTOR = 204; // since setMotor(force) does force*=3 and we want scaled<=614

// Controller state
static volatile bool controlTick = false; // set by ISR
static unsigned long lastSampleMicros = 0;
static int samples[SAMPLES_PER_CYCLE];
static uint8_t sampleCount = 0;

static float Kp = DEFAULT_KP;
static float Ki = DEFAULT_KI;
static float Kd = DEFAULT_KD;
static float integrator = 0.0f;
static float lastError = 0.0f;
static int targetADC = 0;  // target raw ADC value
static bool enabled = false;

// Runtime state for status
static int lastADC = 0;
static int lastDuty = 0; // scaled for setMotor

void ACS712_onTimerTick_ISR()
{
  // Very small: simply set the tick flag and let main loop do work
  controlTick = true;
}

void ACS712_begin()
{
  // Pin definitions centralized in Config.h: ACS712_PIN_SENSE, ACS712_PIN_POWER
  pinMode(ACS712_PIN_SENSE, INPUT); // TODO: confirm pin doesn't conflict with GEARS/WHEEL (see Config.h)
  pinMode(ACS712_PIN_POWER, OUTPUT);
  digitalWrite(ACS712_PIN_POWER, HIGH); // power the ACS712 module
  // initialize timers/state
  lastSampleMicros = micros();
  sampleCount = 0;
  integrator = 0.0f;
  lastError = 0.0f;
  targetADC = zeroADC;  // default target is zero current (at zeroADC)
  enabled = false;
}

/* TODO: make proper implementation of non-blocking ADC sampling using interrupts
void setupADC() {
    // 1. Set Reference to AVcc (5V) and select the channel (e.g., ADC0 / Pin A0)
    ADMUX = (1 << REFS0); 

    // 2. Set ADC Prescaler to 128 (16MHz / 128 = 125kHz sampling clock)
    // This is the most accurate speed for the ATmega32U4 ADC.
    ADCSRA = (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

    // 3. Enable Auto Triggering and the ADC Interrupt
    ADCSRA |= (1 << ADATE) | (1 << ADIE);

    // 4. Set Auto Trigger source to "Free Running Mode" (bits 2:0 are 0)
    ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));

    // 5. Enable the ADC and start the first conversion
    ADCSRA |= (1 << ADEN) | (1 << ADSC);
    
    sei(); // Ensure global interrupts are enabled
}
*/

/* TODO implement simple ADC ISR for non-blocking sampling
      * ISR(ADC_vect) {
            adcSum += ADC; // Add 10-bit result to sum
            adcCount++;
            }
*/

void ACS712_backgroundTask()
{
  unsigned long now = micros();
  // Sample periodically so we get SAMPLES_PER_CYCLE samples per control cycle
  if ((now - lastSampleMicros) >= SAMPLE_INTERVAL_US)
  {
    lastSampleMicros = now;
    if (sampleCount < SAMPLES_PER_CYCLE)
    {
      int v = analogRead(ACS712_PIN_SENSE); // TODO: replace blocking read with non-blocking continuous sampling using ADC interrupts
      // TODO: decide sampling strategy (free-running vs timer-triggered) and document choice
      samples[sampleCount++] = v;
      lastADC = v;
    }
  }

  // Serial command parsing (only if Serial available)
#if DEBUG
  static char cmdBuf[32];
  static uint8_t cmdIdx = 0;
  while (Serial.available())
  {
    char c = Serial.read();
    if (c == '\r' || c == '\n') {
      if (cmdIdx > 0) {
        cmdBuf[cmdIdx] = '\0';
        ACS712_processCommand(cmdBuf);
        cmdIdx = 0;
      }
    } else {
      if (cmdIdx < (sizeof(cmdBuf) - 1)) cmdBuf[cmdIdx++] = c;
    }
  }
#endif
}



void ACS712_update()
{
  if (!controlTick) return; // nothing to do
  controlTick = false;

  // Average samples (if none, use lastADC)
  int avgADC = lastADC;
  if (sampleCount > 0)
  {
    long sum = 0;
    for (uint8_t i = 0; i < sampleCount; ++i) sum += samples[i];
    avgADC = (int)(sum / sampleCount);
    // reset for next cycle
    sampleCount = 0;
  }

  lastADC = avgADC;

  // Safety: shut down if ADC delta exceeds SHUTOFF threshold
  int adcDelta = abs(avgADC - zeroADC);
  if (adcDelta >= SHUTOFF_ADC_DELTA)
  {
    enabled = false;
    setMotor(0);
#if DEBUG
    Serial.print("ERR: SHUTOFF adc="); Serial.print(avgADC);
    Serial.print(" delta="); Serial.println(adcDelta);
#endif
    return;
  }

  if (!enabled)
  {
    // Controller disabled: ensure motor off
    setMotor(0);
    integrator = 0.0f;
    lastError = 0.0f;
    lastDuty = 0;
    return;
  }

  // Control law: PI using raw ADC values (no derivative by default)
  float error = (float)(targetADC - avgADC);
  float dt = (float)CONTROL_INTERVAL_US / 1000000.0f; // seconds

  integrator += error * dt;
  // anti-windup: clamp integrator to reasonable range
  float integLimit = (float)MAX_ADC_DELTA * 10.0f;
  if (integrator > integLimit) integrator = integLimit;
  if (integrator < -integLimit) integrator = -integLimit;

  float u = (Kp * error) + (Ki * integrator) - (Kd * ((error - lastError) / dt));
  lastError = error;

  // Clamp output to MAX_ADC_DELTA range and convert to motor units
  if (u > (float)MAX_ADC_DELTA) u = (float)MAX_ADC_DELTA;
  if (u < -(float)MAX_ADC_DELTA) u = -(float)MAX_ADC_DELTA;

  int motorVal = (int)u;  // direct ADC delta to motor mapping
  lastDuty = motorVal;

  // Apply motor with sign convention: setMotor expects signed value
  setMotor(motorVal);
}

void ACS712_setTargetA(float adcValue)
{
  // Accept raw ADC value (0-1024) directly, no conversion
  targetADC = (int)adcValue;
}

void ACS712_setTargetFromForce(int8_t force)
{
  // Map force (-127 to +127) to ADC range around zeroADC TODO: use const mapper Force values are (-255..255)
  float frac = (float)force / 127.0f;
  int adcTarget = zeroADC + (int)(frac * (float)MAX_ADC_DELTA);
  ACS712_setTargetA((float)adcTarget);
}

void ACS712_enable(bool en)
{
  enabled = en;
  if (!en) { integrator = 0.0f; lastError = 0.0f; setMotor(0); }
}

bool ACS712_isEnabled()
{
  return enabled;
}

void ACS712_setGains(float kp, float ki, float kd)
{
  Kp = kp; Ki = ki; Kd = kd;
}

void ACS712_calibrateZero()
{
  // simple immediate average of available samples or single read
  int v = lastADC;
  if (sampleCount > 0)
  {
    long sum = 0;
    for (uint8_t i = 0; i < sampleCount; ++i) sum += samples[i];
    v = (int)(sum / sampleCount);
  }
  zeroADC = v;
#if DEBUG
  Serial.print("OK CALZ zeroADC="); Serial.println(zeroADC);
#endif
}

void ACS712_processCommand(const char *cmd)
{
  if (cmd == nullptr) return;
  if (strncasecmp(cmd, "T:", 2) == 0)
  {
    float v = atof(cmd + 2);
    ACS712_setTargetA(v);
#if DEBUG
    Serial.print("OK T:"); Serial.println(v);
#endif
    return;
  }
  if (strncasecmp(cmd, "Kp:", 3) == 0)
  {
    float v = atof(cmd + 3); Kp = v;
#if DEBUG
    Serial.print("OK Kp:"); Serial.println(Kp);
#endif
    return;
  }
  if (strncasecmp(cmd, "Ki:", 3) == 0)
  {
    float v = atof(cmd + 3); Ki = v;
#if DEBUG
    Serial.print("OK Ki:"); Serial.println(Ki);
#endif
    return;
  }
  if (strncasecmp(cmd, "Kd:", 3) == 0)
  {
    float v = atof(cmd + 3); Kd = v;
#if DEBUG
    Serial.print("OK Kd:"); Serial.println(Kd);
#endif
    return;
  }
  if (strcasecmp(cmd, "EN") == 0)
  {
    ACS712_enable(true);
#if DEBUG
    Serial.println("OK EN");
#endif
    return;
  }
  if (strcasecmp(cmd, "DIS") == 0)
  {
    ACS712_enable(false);
#if DEBUG
    Serial.println("OK DIS");
#endif
    return;
  }
  if (strcasecmp(cmd, "S") == 0)
  {
#if DEBUG
    Serial.print("S adc:"); Serial.print(lastADC);
    Serial.print(" delta:"); Serial.print(lastADC - zeroADC);
    Serial.print(" target:"); Serial.print(targetADC);
    Serial.print(" duty:"); Serial.print(lastDuty);
    Serial.print(" Kp:"); Serial.print(Kp);
    Serial.print(" Ki:"); Serial.println(Ki);
#endif
    return;
  }
  if (strcasecmp(cmd, "RAW") == 0)
  {
#if DEBUG
    Serial.print("RAW adc:"); Serial.println(lastADC);
#endif
    return;
  }
  if (strcasecmp(cmd, "CALZ") == 0)
  {
    ACS712_calibrateZero();
    return;
  }

#if DEBUG
  Serial.print("ERR Unknown: "); Serial.println(cmd);
#endif
}
