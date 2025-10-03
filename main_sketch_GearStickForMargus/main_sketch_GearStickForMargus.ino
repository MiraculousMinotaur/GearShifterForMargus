#include <Joystick.h>
//Used Libraries inclued as submodules

#define DEBUG 1
#define PEDALS 0
#define GEARS 0 //TODO: conflicts with wheel pins for now.
#define WHEEL 1
#if WHEEL
#define FFB 1 // FFB currently only effects steering
#else
#define FFB 0
#endif

#if DEBUG
#define DEBUG_PRINT(...) Serial.print(__VA_ARGS__)
#define DEBUG_PRINTLN(...) Serial.println(__VA_ARGS__)
#else
#define DEBUG_PRINT(...)
#define DEBUG_PRINTLN(...)
#endif
#if PEDALS
// Pedal Pins
#define ACCELERATOR_PIN A1 
#define BRAKE_PIN A2
#define CLUTCH_PIN A0

// Pedal Calibration
#define ACCELERATOR_MIN_VALUE 320
#define ACCELERATOR_MAX_VALUE 900
#define BRAKE_MIN_VALUE 60
#define BRAKE_MAX_VALUE 950
#define CLUTCH_MIN_VALUE 90
#define CLUTCH_MAX_VALUE 750
#endif
#if GEARS
// Six way pins
#define SIX_WAY_PIN_1 5
#define SIX_WAY_PIN_2 A5
#define SIX_WAY_PIN_3 6
#define SIX_WAY_PIN_4 A4
#define SIX_WAY_PIN_5 7
#define SIX_WAY_PIN_6 A3

// Mode switch Pins
#define MODE_PIN_1 10 //Reverse
#define MODE_PIN_2 11 //Low_Range

// Impulse Switch Pins
#define LOWER_IMPULSE_PIN   9
#define HIGHER_IMPULSE_PIN 8

#define MAIN_STICK_SIZE 6

enum Buttons <int> {
  NORMAL_1 = 0,
  NORMAL_2,
  NORMAL_3,
  NORMAL_4,
  NORMAL_5,
  NORMAL_6,
  REVERSE,
  HIGH_RANGE,
  IMPULSE_1,
  IMPULSE_2,
  LAST_BUTTON
};

const uint8_t SixWayPins[] = {SIX_WAY_PIN_1, SIX_WAY_PIN_2, SIX_WAY_PIN_3, SIX_WAY_PIN_4, SIX_WAY_PIN_5, SIX_WAY_PIN_6};
const uint8_t ModePins[] = {MODE_PIN_1, MODE_PIN_2};
const uint8_t ImpulsePins[] = {LOWER_IMPULSE_PIN, HIGHER_IMPULSE_PIN};

// history
uint8_t prevImpulseState = 0;
uint8_t prevModeState = 0;
uint8_t prevGearState = 0;

void handleGear(&bool currentlyActive, uint8_t prevState, &uint8_t pins, size_t pinCount, Buttons firstButton)
{
    for(int8_t i = pinCount; i > 0; i--)
    {
    if(!digitalRead(pins[i-1]))
    {
      currentlyActive* = true;
      if(!prevState) // gear change can only happen through neutral
      {
        Joystick.pressButton(firstButton + i-1);
        prevState = i;
      }
      else if(prevState != i && digitalRead(pins[prevState-1])){currentlyActive* = false;} // Neutral not detected during gear change
      break;//only read one of the gears
    }
  }
  if(!currentlyActive*)
  {
    for(int8_t i = (pinCount + firstButton); i > firstButton-1;i--){Joystick.releaseButton(i);}
    prevState = 0;
  }
}

#else
    int LAST_BUTTON = 0;
#endif

#if WHEEL
// Encoder Pins
#define ENCODER_PIN_A 2
#define ENCODER_PIN_B 3
// Encoder Limitis
#define ENCODER_MIN_VALUE -6000 // One Full rotation is 2400
#define ENCODER_MAX_VALUE 6000
#if FFB
// Motor Pins
#define MOTOR_PIN_A 9
#define MOTOR_PIN_B 10
#define MOTOR_PIN_ENABLE A0
// Motor Limits
#define MAX_PWM 200 // Going full 255 has higher chance to burn the motor
#define MAX_FORCES 250 // Testing revealed Force MAX values is 250
#define PWM_FORCE_CONVERION MAX_PWM/MAX_FORCES // If better feedback granualarity needed in higer forces implement this conversion
#endif

bool isOutOfRange = false;

volatile int currentPosition = 0;
volatile int8_t oldState = 0;

// Encoder callback function
void tick(void)
{
  int8_t thisState = 0;
  thisState |=  digitalRead(ENCODER_PIN_A);
  thisState |=  digitalRead(ENCODER_PIN_B)<<1;

  switch(thisState)
  {
    case 0:
      currentPosition += (2 == oldState);
      currentPosition -= (1 == oldState);
      break;
    case 1:
      currentPosition += (0 == oldState);
      currentPosition -= (3 == oldState);
      break;
    case 2:
      currentPosition += (3 == oldState);
      currentPosition -= (0 == oldState);
      break;
    case 3:
      currentPosition += (1 == oldState);
      currentPosition -= (2 == oldState);
      break;
    default:
        DEBUG_PRINTLN("ERROR: default");
    }
  DEBUG_PRINTLN(currentPosition);
  oldState = thisState;
}
#if FFB
int32_t forces[2]={0};
Gains gains[2];
EffectParams effectparams[2];
#if DEBUG
  int max_recoded_force = 0;
  int min_recoded_force = 0;
#endif

void beginFFBRequestTimer(void)
{
  cli();
  TCCR3A = 0; //set TCCR1A 0
  TCCR3B = 0; //set TCCR1B 0
  TCNT3  = 0; //counter init
  OCR3A = 400; // 5KHz with 8-fold prescaler TODO can slow down
  TCCR3B |= (1 << WGM32); //open CTC mode
  TCCR3B |= (1 << CS31  ); //set CS11 1(8-fold Prescaler)
  TIMSK3 |= (1 << OCIE3A); //
  sei();
}

void initPWM(void)
{
   // Clear Timer/Counter Control Register A & B
  TCCR1A = 0;
  TCCR1B = 0;

  // Table 14-4. Waveform Generation Mode Bit Description. Page 133
  // Mode:14 - 0 1 1 1 - Fast PWM, 16-bit ICRn TOP
  TCCR1A |= (1 << WGM11) | (0 << WGM10);
  TCCR1B |= (1 << WGM13) | (1 << WGM12);

  // Table 14-5. Clock Select Bit Description. Page 134
  // 0 0 1 ..   /1 = 15.62 kHz PWM
  // 0 1 0 ..   /8 =  1.95 kHz
  // 0 1 1 ..  /64 =    244 Hz
  // 1 0 0 .. /256 =    61 Hz
  TCCR1B |= (0 << CS12) | (0 << CS11) | (1 << CS10); // 0 0 1 ... clkIO/1 (No prescaling)

  // Table 15-7. Compare Output Mode, Phase and Frequency Correct PWM Mode. Page 165
  // COM4A1..0 = 0b10
  //   Cleared on Compare Match when up-counting.
  //   Set on Compare Match when down-counting.
  TCCR1A |= (1 << COM1A1) | (0 << COM1A0);
  TCCR1A |= (1 << COM1B1) | (0 << COM1B0);
  ICR1 = 0x2FF; //~21Khz
}

void setMotor(int force)
{
    force *= 3;//scale for PWM
    if(0 < force)
    {
        OCR1A = 0;
        OCR1B = force;
    }
    else if (0 > force)
    {
        OCR1B = 0;
        OCR1A = -force;
    }
    else
    {
        OCR1B = 0;
        OCR1A = 0;
    }
}

void selfCenter(int wheelOutput)
{
  if(wheelOutput > 0){setMotor(50);}
  else if(wheelOutput < 0){setMotor(-50);}
  else{setMotor(0);}
}
#endif
#endif
#if PEDALS || WHEEL || FFB
template <typename T> T limit(T value, T min, T max)
{
  if(value < min){return min;}
  if(value > max){return max;}
  return value;
}
#endif

Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_JOYSTICK,
  LAST_BUTTON, 0,                  // Button Count, Hat Switch Count
  true, true, true,     // X and Y, but no Z Axis
  false, false, false,   //  Rx, Ry, no Rz
  false, false,          // No rudder or throttle
  true, true, false);    // No accelerator, brake, or steering

void setup() {
#if DEBUG
  Serial.begin(9600);
#endif
#if PEDALS
  // Initalize pedals
  Joystick.setAcceleratorRange(ACCELERATOR_MIN_VALUE, ACCELERATOR_MAX_VALUE);
  Joystick.setBrakeRange(BRAKE_MIN_VALUE, BRAKE_MAX_VALUE);
  Joystick.setZAxisRange(CLUTCH_MIN_VALUE, CLUTCH_MAX_VALUE);
#endif
#if GEARS
  // Prepare InputPins
  for(int8_t i = sizeof(SixWayPins); i > -1; i--) {pinMode(SixWayPins[i],INPUT_PULLUP);}
  for(int8_t i = sizeof(ModePins); i > -1; i--)   {pinMode(ModePins[i],INPUT_PULLUP);}
  for(int8_t i = sizeof(ImpulsePins); i > -1; i--){pinMode(ImpulsePins[i],INPUT_PULLUP);}
#endif
#if WHEEL
  pinMode(ENCODER_PIN_A, INPUT_PULLUP);
  pinMode(ENCODER_PIN_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_A),tick,CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_PIN_B),tick,CHANGE);
  Joystick.setXAxisRange(ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
#if FFB
  pinMode(MOTOR_PIN_A, OUTPUT);
  pinMode(MOTOR_PIN_B, OUTPUT);
  pinMode(MOTOR_PIN_ENABLE, OUTPUT);
  digitalWrite(MOTOR_PIN_ENABLE, HIGH);
  beginFFBRequestTimer();
  initPWM();
  setMotor(0);

  effectparams[0].springMaxPosition = ENCODER_MAX_VALUE;
  effectparams[0].springPosition = currentPosition;
  effectparams[1].springMaxPosition = 255;
  effectparams[1].springPosition = 0;

  Joystick.setGains(gains);
#endif
#endif
  // Initialize Joystick Library
  Joystick.begin(true);
}
#if FFB
ISR(TIMER3_COMPA_vect){Joystick.getUSBPID();}
#endif

void loop() 
{
#if DEBUG
  delay(100);
#endif
#if PEDALS
  //Pedal Handeling
  int pedal = 0;
  pedal = analogRead(ACCELERATOR_PIN);
  DEBUG_PRINT("Accelerator: ");
  DEBUG_PRINT(pedal);
  pedal = limit(pedal, ACCELERATOR_MIN_VALUE, ACCELERATOR_MAX_VALUE);
  Joystick.setAccelerator(pedal);
  DEBUG_PRINT(" ");
  DEBUG_PRINT(pedal);
  
  pedal = 0;
  pedal = analogRead(BRAKE_PIN);
  DEBUG_PRINT(" Break: ");
  DEBUG_PRINT(pedal);

  pedal = limit(pedal, BRAKE_MIN_VALUE, BRAKE_MAX_VALUE);
  Joystick.setBrake(pedal);
  DEBUG_PRINT(" ");
  DEBUG_PRINT(pedal);

  pedal = 0;
  pedal = analogRead(CLUTCH_PIN);
  DEBUG_PRINT(" Clutch: ");
  DEBUG_PRINT(pedal);

  pedal = limit(pedal, CLUTCH_MIN_VALUE, CLUTCH_MAX_VALUE);
  Joystick.setZAxis(pedal);
  DEBUG_PRINT(" ");
  DEBUG_PRINTLN(pedal);
#endif
#if GEARS
  bool inImpulse = false;
  bool inMode = false;
  bool inGear = false;
  handleGear(inImpulse, prevImpulseState, ImpulsePins, sizeof(ImpulsePins), IMPULSE_1);
  handleGear(inMode, prevModeState, ModePins, sizeof(ModePins), REVERSE);
  handleGear(inGear, prevGearState, SixWayPins, sizeof(SixWayPins), NORMAL_1);
#endif
#if WHEEL
	int wheelOutput = limit(currentPosition, ENCODER_MIN_VALUE, ENCODER_MAX_VALUE);
  DEBUG_PRINT("Wheel Output: ");
  DEBUG_PRINT(wheelOutput);
  Joystick.setXAxis(wheelOutput);

#if FFB
  effectparams[0].springPosition = wheelOutput;
  Joystick.setEffectParams(effectparams);
  Joystick.getForce(forces);
#if DEBUG
  if(forces[0] > max_recoded_force){max_recoded_force = forces[0];}
  if(forces[0] < min_recoded_force){min_recoded_force = forces[0];}
  DEBUG_PRINT(" MAX Force: ");
  DEBUG_PRINT(max_recoded_force);
  DEBUG_PRINT(" MIN Force: ");
  DEBUG_PRINT(min_recoded_force);
  DEBUG_PRINT(" RAW Force: ");
  DEBUG_PRINT(forces[0]);
#endif
  int force = limit((int)forces[0], -MAX_PWM, MAX_PWM);
  DEBUG_PRINT(" Force: ");
  DEBUG_PRINTLN(force);
  setMotor(force);
  //selfCenter(wheelOutput);
#endif
#endif
}
