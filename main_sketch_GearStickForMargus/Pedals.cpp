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

ads_state_t currentAdsState = IDLE;

ads_state_t get_Ads_State()
{
  return currentAdsState;
}

// Made as a function incase I want to time it better in main loop or something, but for now it's just called from Pedals_update()
void Report_Pedals(void)
{
  // Joystick has it's own constaint.
  Joystick.setRxAxis((int)adsValues[ADS_CH_ACCEL]);
  Joystick.setRyAxis((int)adsValues[ADS_CH_BRAKE]);
  Joystick.setZAxis((int)adsValues[ADS_CH_CLUTCH]);
}

void Pedals_update(void)
{
  switch (currentAdsState)
  {
  case IDLE:
    ads.startADCReading((MUX_BY_CHANNEL[currentChannel]), /*multishot=*/false);
    currentAdsState = WAITING_ON_CONVERSION;
    break;
  case WAITING_ON_CONVERSION:
    if(!ads.conversionComplete()){return;} // If conversion not complete, skip this update cycle.
    currentAdsState = CONVERTED;
    break;
  case CONVERTED:
    adsValues[currentChannel] = ads.getLastConversionResults();
    // Cycle to the next channel (0 -> 1 -> 2 -> 0)
    currentChannel = currentChannel < 2?(currentChannel + 1) : 0 ; //
    currentAdsState = IDLE;
    Report_Pedals();
  default:
    break;
  }
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
