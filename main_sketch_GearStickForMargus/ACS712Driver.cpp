#include "ACS712Driver.h"
#include "Wheel.h"
#include "Config.h"
#include "Utils.h"
#include "motor.h"

//TODO: don't use floats for better performance on 8-bit MCU
// Configurable parameters
static const float DEFAULT_KP = 1.0f;
static const float DEFAULT_KI = 0.1f;
static const float DEFAULT_KD = 0.0f;
static const float MAX_AMPS = 10.0f;       // limit (user requirement)
static const float SHUTOFF_AMPS = 14.0f;   // immediate shutoff if exceeded

// ADC calibration defaults (10-bit ADC values) // TODO: use mA rather than A for better precision and non float calculations
static int zeroADC = 513;
static const float ADC_PER_A_POS = 13.0f;  // +1A => 526 (Δ+13)
static const float ADC_PER_A_NEG = 14.0f;  // -1A => 499 (Δ-14)

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
static float targetAmps = 0.0f;
static bool enabled = false;

// Runtime state for status
static float lastMeasuredAmps = 0.0f;
static int lastADC = 0;
static int lastDuty = 0; // scaled for setMotor

void ACS712_onTimerTick_ISR()
{
  // Very small: simply set the tick flag and let main loop do work
  controlTick = true;
}

void ACS712_begin()
{
  pinMode(A5, INPUT);// TODO: move Pin definitions to header
  pinMode(A4, OUTPUT);
  digitalWrite(A4, HIGH); // power the ACS712 module
  // initialize timers/state
  lastSampleMicros = micros();
  sampleCount = 0;
  integrator = 0.0f;
  lastError = 0.0f;
  targetAmps = 0.0f;
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
      int v = analogRead(A5); // TODO: move Pin definitions to header
      //TODO: replace blocking read with non-blocking continuous sampling using ADC interrupts
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

static float adcToAmps(int adc)
{
  int delta = adc - zeroADC;
  if (delta >= 0)
    return (float)delta / ADC_PER_A_POS;
  else
    return (float)delta / ADC_PER_A_NEG;
}

static int ampsToSetMotor(float amps)
{
  // Map -MAX_AMPS..MAX_AMPS -> -MAX_SETMOTOR..MAX_SETMOTOR
  if (amps > MAX_AMPS) amps = MAX_AMPS;
  if (amps < -MAX_AMPS) amps = -MAX_AMPS;
  float frac = amps / MAX_AMPS;
  int val = (int)roundf(frac * (float)MAX_SETMOTOR);
  return limitVal(val, -MAX_SETMOTOR, MAX_SETMOTOR);
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
  // TODO: remove ADC conversions to AMPS, use DIRECT ADC values in control loop for better performance
  lastMeasuredAmps = adcToAmps(avgADC);

  // Safety: shut down if exceeds SHUTOFF
  if (abs(lastMeasuredAmps) >= SHUTOFF_AMPS)
  {
    enabled = false;
    setMotor(0);
#if DEBUG
    Serial.print("ERR: SHUTOFF measuredA="); Serial.println(lastMeasuredAmps);
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

  // Control law: PI (no derivative by default)
  float error = targetAmps - lastMeasuredAmps;
  float dt = (float)CONTROL_INTERVAL_US / 1000000.0f; // seconds

  integrator += error * dt;
  // anti-windup
  float integLimit = MAX_AMPS * 10.0f; // heuristic
  if (integrator > integLimit) integrator = integLimit;
  if (integrator < -integLimit) integrator = -integLimit;

  float u = (Kp * error) + (Ki * integrator) - (Kd * ((error - lastError) / dt));
  lastError = error;

  // Desired actuation in amps -> convert to setMotor units
  // Clamp u to MAX_AMPS range
  if (u > MAX_AMPS) u = MAX_AMPS;
  if (u < -MAX_AMPS) u = -MAX_AMPS;

  int motorVal = ampsToSetMotor(u);
  lastDuty = motorVal;

  // Apply motor with sign convention: setMotor expects signed value
  setMotor(motorVal);
}

void ACS712_setTargetA(float amps)
{
  if (amps > MAX_AMPS) amps = MAX_AMPS;
  if (amps < -MAX_AMPS) amps = -MAX_AMPS;
  targetAmps = amps;
}

void ACS712_setTargetFromForce(int8_t force)
{
  float frac = (float)force / 255.0f;
  ACS712_setTargetA(frac * MAX_AMPS);
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
    Serial.print(" amps:"); Serial.print(lastMeasuredAmps);
    Serial.print(" target:"); Serial.print(targetAmps);
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
  if (strncasecmp(cmd, "CALP:", 5) == 0)
  {
    int v = atoi(cmd + 5); // raw adc for +1A
    // derive ADC_PER_A_POS from given v and zeroADC
    if (v != zeroADC)
    {
      float slope = (float)(v - zeroADC);
      if (slope > 0.0f) { /* set adjustable if required */ }
#if DEBUG
      Serial.print("OK CALP:"); Serial.println(v);
#endif
    }
    return;
  }
  if (strncasecmp(cmd, "CALN:", 5) == 0)
  {
    int v = atoi(cmd + 5);
#if DEBUG
    Serial.print("OK CALN:"); Serial.println(v);
#endif
    return;
  }

#if DEBUG
  Serial.print("ERR Unknown: "); Serial.println(cmd);
#endif
}
