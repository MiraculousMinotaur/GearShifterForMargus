#include "ACS712Driver.h"
#include "Wheel.h"
#include "Config.h"
#include "Utils.h"
#include "motor.h"
#include "FixedPoint.h"

// Configurable parameters (raw ADC units, no conversion)
static const int16_t DEFAULT_KP_Q8 = (int16_t)(1 * SCALE_Q8); // 1.0 -> 256
static const int16_t DEFAULT_KI_Q8 = (int16_t)(26); // ~0.1 * 256 = 25.6 -> 26
static const int16_t DEFAULT_KD_Q8 = (int16_t)(0);
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

static int16_t Kp_q8 = DEFAULT_KP_Q8;
static int16_t Ki_q8 = DEFAULT_KI_Q8;
static int16_t Kd_q8 = DEFAULT_KD_Q8;
static int32_t integrator_q = 0;
static int16_t lastError_i = 0;
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
  integrator_q = 0;
  lastError_i = 0;
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
    integrator_q = 0;
    lastError_i = 0;
    lastDuty = 0;
    return;
  }

  // Control law: PI using raw ADC values (integer fixed-point Q8 gains)
  int32_t error = (int32_t)targetADC - (int32_t)avgADC; // ADC units

  // Integrator (per-control-tick interpretation)
  integrator_q += error;
  // anti-windup: clamp integrator to reasonable range
  int32_t integLimit = (int32_t)MAX_ADC_DELTA * 10;
  if (integrator_q > integLimit) integrator_q = integLimit;
  if (integrator_q < -integLimit) integrator_q = -integLimit;

  // Compute control output in Q8 domain: u_q8 = Kp_q8*error + (Ki_q8*integrator)/SCALE_Q8 - (Kd_q8*(error-lastError))/SCALE_Q8
  int64_t termP = (int64_t)Kp_q8 * (int64_t)error; // Q8 * int -> Q8*int
  int64_t termI = ((int64_t)Ki_q8 * (int64_t)integrator_q) / (int64_t)SCALE_Q8; // bring back to Q8
  int64_t termD = 0;
  int32_t derror = error - (int32_t)lastError_i;
  termD = ((int64_t)Kd_q8 * (int64_t)derror) / (int64_t)SCALE_Q8;

  int64_t u_q8 = termP + termI - termD;
  int32_t u = (int32_t)(u_q8 / (int64_t)SCALE_Q8); // back to ADC units
  lastError_i = (int16_t)error;

  // Clamp output to MAX_ADC_DELTA range and convert to motor units
  if (u > MAX_ADC_DELTA) u = MAX_ADC_DELTA;
  if (u < -MAX_ADC_DELTA) u = -MAX_ADC_DELTA;

  int motorVal = (int)u;  // direct ADC delta to motor mapping
  lastDuty = motorVal;

  // Apply motor with sign convention: setMotor expects signed value
  setMotor(motorVal);
}

void ACS712_setTargetA(int adcValue)
{
  // Accept raw ADC value (0-1024) directly, no conversion
  targetADC = adcValue;
}

void ACS712_setTargetFromForce(int force)
{
  // Map force (-MAX_FORCES .. +MAX_FORCES) to ADC range around zeroADC
  // integer mapping: adcTarget = zeroADC + force * MAX_ADC_DELTA / MAX_FORCES
#if defined(MAX_FORCES)
  int adcTarget = zeroADC + ((int)force * MAX_ADC_DELTA) / MAX_FORCES;
#else
  // fallback to 127 if MAX_FORCES not defined
  int adcTarget = zeroADC + ((int)force * MAX_ADC_DELTA) / 127;
#endif
  ACS712_setTargetA(adcTarget);
}

void ACS712_enable(bool en)
{
  enabled = en;
  if (!en) { integrator_q = 0; lastError_i = 0; setMotor(0); }
}

bool ACS712_isEnabled()
{
  return enabled;
}

void ACS712_setGains_q8(int16_t kp_q8, int16_t ki_q8, int16_t kd_q8)
{
  Kp_q8 = kp_q8;
  Ki_q8 = ki_q8;
  Kd_q8 = kd_q8;
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
    int v = atoi(cmd + 2);
    ACS712_setTargetA(v);
#if DEBUG
    Serial.print("OK T:"); Serial.println(v);
#endif
    return;
  }
  if (strncasecmp(cmd, "Kp:", 3) == 0)
  {
    q8_t v = parse_fixed_q8(cmd + 3);
    Kp_q8 = v;
#if DEBUG
    Serial.print("OK Kp_q8:"); Serial.println((int)Kp_q8);
#endif
    return;
  }
  if (strncasecmp(cmd, "Ki:", 3) == 0)
  {
    q8_t v = parse_fixed_q8(cmd + 3);
    Ki_q8 = v;
#if DEBUG
    Serial.print("OK Ki_q8:"); Serial.println((int)Ki_q8);
#endif
    return;
  }
  if (strncasecmp(cmd, "Kd:", 3) == 0)
  {
    q8_t v = parse_fixed_q8(cmd + 3);
    Kd_q8 = v;
#if DEBUG
    Serial.print("OK Kd_q8:"); Serial.println((int)Kd_q8);
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
    Serial.print(" Kp_q8:"); Serial.print((int)Kp_q8);
    Serial.print(" Ki_q8:"); Serial.println((int)Ki_q8);
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
