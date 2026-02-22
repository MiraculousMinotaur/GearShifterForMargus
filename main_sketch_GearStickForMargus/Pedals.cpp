#include "Pedals.h"

#if PEDALS

#include "DebugManager.h"

// ADS1115 instance (using I2C)
Adafruit_ADS1115 ads;

// Array to store raw ADS values for channels 0 (accel), 1 (brake), 2 (clutch), 3 (ref)
// ADS1115 returns int16_t values (-32768 to 32767, with 0 at center)
static int16_t adsValues[3] = {0, 0, 0};

// Static channel index for cycling through channels
// Initialized to 3 so the first call reads ch3 and starts ch0
static uint8_t currentChannel = 3;

void Pedals_begin()
{
  // ADS1115 is already initialized in main_sketch setup via I2C power enable and Wire.begin()
  // This function is called after device initialization, so ads.begin() has already been called.
  
  // Set ranges for Joystick output using calibration values as min/max endpoints
  Joystick.setRxAxisRange(ACCELERATOR_MIN_VALUE, ACCELERATOR_MAX_VALUE);
  Joystick.setRyAxisRange(BRAKE_MIN_VALUE, BRAKE_MAX_VALUE);
  Joystick.setZAxisRange(CLUTCH_MIN_VALUE, CLUTCH_MAX_VALUE);

  // Initialize ADS values with calibration midpoints for safety
  // These will be overwritten as soon as continuous conversions begin
  adsValues[ADS_CH_ACCEL] = ACCELERATOR_MIN_VALUE;
  adsValues[ADS_CH_BRAKE] = BRAKE_MIN_VALUE;
  adsValues[ADS_CH_CLUTCH] = CLUTCH_MIN_VALUE;

  // Initial channel cycling setup: start continuous conversion on channel 0
  // Start continuous conversions on pedal channels (0,1,2) and reference (3)
  // Use single-ended reads on channels 0-3
  // Note: ADS1115 can only measure one channel in continuous mode at a time,
  // so we cycle through channels by starting on channel 0 and reading the last result
  // The scheduler will cycle through channels in Pedals_update()
  currentChannel = ADS_CH_BRAKE;  // Will cycle to 0 in first Pedals_update() call
  ads.startADCReading((MUX_BY_CHANNEL[currentChannel]), /*multishot=*/false);  // Start continuous on accel channel
}

void Pedals_update()
{
  if(!ads.conversionComplete()){return;} // If conversion not complete, skip this update cycle.
  // Read the last conversion result (from the previous channel)
  // ADS1115 returns int16_t where center (0 value) is at max_int16/2 = 32767/2 ≈ 16384
  adsValues[currentChannel] = ads.getLastConversionResults();

  // Cycle to the next channel (0 -> 1 -> 2 -> 3 -> 0)
  currentChannel = currentChannel < 2?(currentChannel + 1) : 0 ; //
  
  ads.startADCReading((MUX_BY_CHANNEL[currentChannel]), /*multishot=*/false);

  // Update Joystick outputs with constrained raw 16-bit values
  // Constrain each raw value to its calibration range [MIN_VALUE, MAX_VALUE]
  int16_t accelValue = constrain(adsValues[ADS_CH_ACCEL], ACCELERATOR_MIN_VALUE, ACCELERATOR_MAX_VALUE);
  Joystick.setRxAxis((int)accelValue);

  int16_t brakeValue = constrain(adsValues[ADS_CH_BRAKE], BRAKE_MIN_VALUE, BRAKE_MAX_VALUE);
  Joystick.setRyAxis((int)brakeValue);

  int16_t clutchValue = constrain(adsValues[ADS_CH_CLUTCH], CLUTCH_MIN_VALUE, CLUTCH_MAX_VALUE);
  Joystick.setZAxis((int)clutchValue);
  
  // Channel 3 (reference) is read and stored for future use (e.g., calibration, diagnostics)
  // Currently not used in output but available via adsValues[ADS_CH_REF]
}

// ===== Debug Report Function =====
#if DEBUG
void Pedals_reportDebug(struct DebugTelemetry_t *tel)
{
  if (tel) {
    tel->pedals_accel = adsValues[ADS_CH_ACCEL];
    tel->pedals_brake = adsValues[ADS_CH_BRAKE];
    tel->pedals_clutch = adsValues[ADS_CH_CLUTCH];
  }
}
#endif

#endif // PEDALS
