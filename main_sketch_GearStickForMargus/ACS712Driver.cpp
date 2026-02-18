#include "ACS712Driver.h"
#include "Wheel.h"
#include "Config.h"
#include "Utils.h"
#include "motor.h"
#include "FixedPoint.h"
#include "DebugManager.h"

// Configurable parameters (raw ADC units, no conversion)
static const int16_t DEFAULT_KP_Q8 = (int16_t)(1 * SCALE_Q8); // 1.0 -> 256
static const int16_t DEFAULT_KI_Q8 = (int16_t)(26); // ~0.1 * 256 = 25.6 -> 26
static const int16_t DEFAULT_KD_Q8 = (int16_t)(0);
static const int16_t MAX_ADC_DELTA = 130;      // max ADC delta from zero (corresponds to ~10A)
static const int16_t SHUTOFF_ADC_DELTA = 156;  // emergency shutoff threshold (corresponds to ~12A)

// ADC calibration (10-bit ADC value at zero current)
static uint16_t zeroADC = 513;

// Sampling & timing // TODO sampling tuning will be handled by register HW config handled in SetupADC() function removed unneccesary values
static const uint16_t CONTROL_HZ = 240; // default control rate (Hz)
static const uint32_t CONTROL_INTERVAL_US = 1000000UL / CONTROL_HZ;
static const uint8_t SAMPLES_PER_CYCLE = 4;
static const uint32_t SAMPLE_INTERVAL_US = CONTROL_INTERVAL_US / SAMPLES_PER_CYCLE; // approx spacing

// Motor limits tied to setMotor scaling
static const int16_t MAX_SETMOTOR = 204; // since setMotor(force) does force*=3 and we want scaled<=614

// Controller state
// Note: sampling is done in ISR accumulators (adcSum/adcCount). Do not use samples[]/sampleCount.
// static int samples[SAMPLES_PER_CYCLE];
// static uint8_t sampleCount = 0;

// ISR accumulators (16-bit as requested)
static volatile uint16_t adcSum = 0;
static volatile uint16_t adcCount = 0;

static int16_t Kp_q8 = DEFAULT_KP_Q8;
static int16_t Ki_q8 = DEFAULT_KI_Q8;
static int16_t Kd_q8 = DEFAULT_KD_Q8;
static int32_t integrator_q = 0;
static int16_t lastError_i = 0;
static uint16_t targetADC = 0;  // target raw ADC value
static bool enabled = false;

// Runtime state for status
static uint16_t lastADC = 0;
static int16_t lastDuty = 0; // scaled for setMotor

void ACS712_begin()
{
  // Pin definitions centralized in Config.h: ACS712_PIN_SENSE, ACS712_PIN_POWER
  pinMode(ACS712_PIN_SENSE, INPUT); // TODO: confirm pin doesn't conflict with GEARS/WHEEL (see Config.h)
  pinMode(ACS712_PIN_POWER, OUTPUT);
  digitalWrite(ACS712_PIN_POWER, HIGH); // power the ACS712 module
  // initialize timers/state
  // clear ISR accumulators
  noInterrupts();
  adcSum = 0;
  adcCount = 0;
  interrupts();
  integrator_q = 0;
  lastError_i = 0;
  targetADC = zeroADC;  // default target is zero current (at zeroADC)
  enabled = false;
  // Configure and start free-running ADC on ACS712_PIN_SENSE
  // Derive ADC channel from analog pin macro if available
#ifdef analogPinToChannel
  setupADC(analogPinToChannel(ACS712_PIN_SENSE));
#else
  // Fallback: assume analog pins A0.. map to channels 0..
  setupADC(ACS712_PIN_SENSE - A0);
#endif
  startADC();
}

// ADC ISR: accumulate 10-bit ADC results into adcSum and adcCount (keep ISR minimal)
ISR(ADC_vect)
{
  uint16_t v = ADC; // read ADC (10-bit result in 16-bit register)
  adcSum += v;
  adcCount++;
}

// Configure ADC for free-running on given channel (channel = ADC channel number)
void setupADC(uint8_t channel)
{
  // Select AVcc as reference and channel
  ADMUX = (1 << REFS0) | (channel & 0x0F);

  // Free running: clear ADTS bits
  ADCSRB &= ~((1 << ADTS2) | (1 << ADTS1) | (1 << ADTS0));

  // Prescaler /128, enable ADC, enable auto trigger and ADC interrupt
  ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADATE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
}

void startADC()
{
  // Start first conversion (free-running)
  ADCSRA |= (1 << ADSC);
}

void stopADC()
{
  // Disable ADC conversions and interrupt
  ADCSRA &= ~((1 << ADEN) | (1 << ADSC) | (1 << ADIE));
}

// Non-atomic snapshot: copies and clears ISR accumulators. Caller must wrap with noInterrupts()/interrupts() if atomicity required.
void ACS712_snapshotAndClear(uint16_t *sum, uint16_t *count)
{
  if (sum) {
    *sum = adcSum;
    adcSum = 0;
  }
  if (count) {
    *count = adcCount;
    adcCount = 0;
  }
}

void ACS712_setLastADC(uint16_t v)
{
  lastADC = v;
}

void ACS712_update()
{
  // Safety: shut down if ADC delta exceeds SHUTOFF threshold
  int16_t adcDelta = (lastADC > zeroADC) ? (lastADC - zeroADC) : (zeroADC - lastADC);
  if (adcDelta >= SHUTOFF_ADC_DELTA)
  {
    enabled = false;
    setMotor(0);
    Serial.print("ERR: SHUTOFF adc="); Serial.print((int)lastADC);
    Serial.print(" delta="); Serial.println((int)adcDelta);
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
  int32_t error = (int32_t)targetADC - (int32_t)lastADC; // ADC units

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

  int16_t motorVal = (int16_t)u;  // direct ADC delta to motor mapping
  lastDuty = motorVal;

  // Apply motor with sign convention: setMotor expects signed value
  setMotor(motorVal);
}

void ACS712_setTargetA(uint16_t adcValue)
{
  // Accept raw ADC value (0-1024) directly, no conversion
  targetADC = adcValue;
}

void ACS712_setTargetFromForce(int16_t force)
{
  // Map force (-MAX_FORCES .. +MAX_FORCES) to ADC range around zeroADC
  // integer mapping: adcTarget = zeroADC + force * MAX_ADC_DELTA / MAX_FORCES
#if defined(MAX_FORCES)
  int32_t adcTarget = (int32_t)zeroADC + (((int32_t)force * (int32_t)MAX_ADC_DELTA) / (int32_t)MAX_FORCES);
#else
  // fallback to 127 if MAX_FORCES not defined
  int32_t adcTarget = (int32_t)zeroADC + (((int32_t)force * (int32_t)MAX_ADC_DELTA) / 127);
#endif
  ACS712_setTargetA((uint16_t)adcTarget);
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
  // -1 means don't change that parameter
  if (kp_q8 >= 0) Kp_q8 = kp_q8;
  if (ki_q8 >= 0) Ki_q8 = ki_q8;
  if (kd_q8 >= 0) Kd_q8 = kd_q8;
}

void ACS712_calibrateZero()
{
  zeroADC = lastADC;
}

// ===== Debug Report Function =====
void ACS712_reportDebug(struct DebugTelemetry_t *tel)
{
  if (tel) {
    tel->acs_raw_adc = lastADC;
    tel->acs_delta = (int16_t)((int32_t)lastADC - (int32_t)zeroADC);
    tel->acs_target = targetADC;
    tel->acs_current_duty = lastDuty;
    tel->acs_kp_q8 = Kp_q8;
    tel->acs_ki_q8 = Ki_q8;
    tel->acs_kd_q8 = Kd_q8;
  }
}
