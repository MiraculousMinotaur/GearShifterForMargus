# Hardware Configuration & Customization Guide

This guide explains how to adapt the **GearShifterForMargus** firmware for custom sim-racing and sim-trucking hardware, different button configurations, alternative pedal sensors, steering encoders, and motor drivers.

---

## 1. Feature Flags & Subsystem Toggles

All subsystems can be independently enabled or disabled in [`main_sketch_GearStickForMargus/Config.h`](../main_sketch_GearStickForMargus/Config.h):

```c
#define DEBUG  0   // Set 1 to stream live sensor CSV over USB Serial (115200 baud)
#define PEDALS 1   // 1 = Enable ADS1115 pedals (Throttle, Brake, Clutch)
#define GEARS  1   // 1 = Enable MCP23017 gear shifter and toggles
#define WHEEL  1   // 1 = Enable steering wheel encoder and limits
#define FFB    1   // 1 = Enable closed-loop motor force feedback (requires WHEEL 1)
```

**Use Cases:**
- Building a **standalone H-Shifter**: set `PEDALS 0`, `WHEEL 0`, `FFB 0`.
- Building a **standalone Pedal Set**: set `GEARS 0`, `WHEEL 0`, `FFB 0`.
- Building a **standalone FFB Wheel base**: set `PEDALS 0`, `GEARS 0`.

---

## 2. Customizing Gear Shifter & Button Configurations

All switch inputs are read via an **MCP23017** 16-channel I2C I/O expander at address `0x20`.

### MCP23017 Pin Index Mapping
The MCP23017 has two 8-bit ports. In this firmware, they are indexed from 0 to 15:
- **Port A (GPA0 – GPA7):** Pin indices `0` through `7`.
- **Port B (GPB0 – GPB7):** Pin indices `8` through `15`.

### Default Wiring & Button Mapping
| MCP Pin Index | Port Pin | Hardware Input | Code Symbol | Joystick Button |
|---|---|---|---|---|
| `10` | GPB2 | 1st Gear | `SIX_WAY_PIN_1` | Button 0 (`NORMAL_1`) |
| `11` | GPB3 | 2nd Gear | `SIX_WAY_PIN_2` | Button 1 (`NORMAL_2`) |
| `12` | GPB4 | 3rd Gear | `SIX_WAY_PIN_3` | Button 2 (`NORMAL_3`) |
| `13` | GPB5 | 4th Gear | `SIX_WAY_PIN_4` | Button 3 (`NORMAL_4`) |
| `14` | GPB6 | 5th Gear | `SIX_WAY_PIN_5` | Button 4 (`NORMAL_5`) |
| `15` | GPB7 | 6th Gear | `SIX_WAY_PIN_6` | Button 5 (`NORMAL_6`) |
| `0` | GPA0 | Reverse Switch | `MODE_PIN_1` | Button 6 (`REVERSE`) |
| `1` | GPA1 | Range Switch (High/Low) | `MODE_PIN_2` | Button 7 (`HIGH_RANGE`) |
| `6` | GPA6 | Splitter Toggle (Lower) | `LOWER_IMPULSE_PIN` | Button 8 (`IMPULSE_1`) |
| `7` | GPA7 | Splitter Toggle (Higher)| `HIGHER_IMPULSE_PIN`| Button 9 (`IMPULSE_2`) |

> [!NOTE]
> **Switch Electrical Logic:**
> By default, all 16 pins of the MCP23017 are initialized with internal pull-ups enabled (`INPUT_PULLUP`).
> Switches must be wired between the respective **MCP23017 pin and Ground (GND)**.
> - Switch closed to GND = `LOW` = **Pressed (Active)**.
> - Switch open = `HIGH` = **Released (Inactive)**.

---

### How to Adjust Pin Assignments

If your physical PCB or wiring uses different pins on the MCP23017, modify the definitions in [`main_sketch_GearStickForMargus/Config.h`](../main_sketch_GearStickForMargus/Config.h) or [`Gears.h`](../main_sketch_GearStickForMargus/Gears.h):

```c
// Example: In Config.h
#define MCP_GEAR_SIXWAY_FIRST_PIN  10  // Starting pin index for 6-way gate
#define MCP_GEAR_MODE_FIRST_PIN     0  // Starting pin index for Mode switches
#define MCP_GEAR_IMPULSE_FIRST_PIN  6  // Starting pin index for Splitter/Impulse toggles
```

Or reassign individual pins in [`Gears.h`](../main_sketch_GearStickForMargus/Gears.h):
```c
#define SIX_WAY_PIN_1 8   // Move 1st gear to GPB0 (pin 8)
#define SIX_WAY_PIN_2 9   // Move 2nd gear to GPB1 (pin 9)
...
```

---

### How to Add More Gears (e.g., 7th & 8th Gear) or Custom Buttons

If your shifter has 7 or 8 gate positions, or if you want to add auxiliary buttons (Horn, Engine Start, Flashers, Retarder, etc.):

#### Step 1: Update the Button Count in `JoystickGlue.cpp`
In [`main_sketch_GearStickForMargus/JoystickGlue.cpp`](../main_sketch_GearStickForMargus/JoystickGlue.cpp):
```cpp
#if GEARS
const uint8_t LAST_BUTTON = 12; // Increase total button count (e.g. from 10 to 12)
#else
const uint8_t LAST_BUTTON = 0;
#endif
```

#### Step 2: Expand the Button Enum in `Gears.h`
In [`main_sketch_GearStickForMargus/Gears.h`](../main_sketch_GearStickForMargus/Gears.h):
```cpp
// Define the new pins on unused MCP pins (e.g. GPA2=2, GPA3=3)
#define SIX_WAY_PIN_7 2
#define SIX_WAY_PIN_8 3

enum Buttons_e {
  NORMAL_1 = 0,
  NORMAL_2,
  NORMAL_3,
  NORMAL_4,
  NORMAL_5,
  NORMAL_6,
  NORMAL_7,     // Added Gear 7
  NORMAL_8,     // Added Gear 8
  REVERSE,
  HIGH_RANGE,
  IMPULSE_1,
  IMPULSE_2,
  LAST_BUTTON
};
```

#### Step 3: Update the Pin Array in `Gears.cpp`
In [`main_sketch_GearStickForMargus/Gears.cpp`](../main_sketch_GearStickForMargus/Gears.cpp):
```cpp
const uint8_t SixWayPins[] = {
  SIX_WAY_PIN_1, SIX_WAY_PIN_2, SIX_WAY_PIN_3, 
  SIX_WAY_PIN_4, SIX_WAY_PIN_5, SIX_WAY_PIN_6,
  SIX_WAY_PIN_7, SIX_WAY_PIN_8
};
```
The helper function `handleGear()` automatically sizes itself using `sizeof(SixWayPins)` and will service all 8 gears!

---

## 3. Customizing Pedals & Calibration

The pedal set uses an external I2C ADC connected at address `0x48`. Both the **ADS1115 (16-bit)** and the pin-compatible **ADS1015 (12-bit)** are supported by the `Adafruit_ADS1X15` library.

> [!TIP]
> **ADS1115 (16-bit) vs ADS1015 (12-bit):**
> - If your breakout board has an **ADS1115**, raw conversion values range up to `32767` (16-bit signed).
> - If your board has an **ADS1015**, the ADC resolution is 12-bit (values shifted by 4 bits, ranging up to `2047`).
> - If swapping to an ADS1015 in code, define `Adafruit_ADS1015 ads;` in [`Pedals.cpp`](../main_sketch_GearStickForMargus/Pedals.cpp) and update `ADS1115_DATARATE` to an ADS1015 data rate (e.g. `RATE_ADS1015_1600SPS`) in [`Config.h`](../main_sketch_GearStickForMargus/Config.h).

### Wiring
- **VDD:** 5V
- **GND:** Ground
- **SCL / SDA:** Arduino Pins 3 (SCL) and 2 (SDA)
- **A0:** Brake Potentiometer / Load Cell Amplifier signal
- **A1:** Accelerator Potentiometer / Hall sensor signal
- **A2:** Clutch Potentiometer / Hall sensor signal

### Calibration Walkthrough
Every DIY pedal assembly has unique physical travel and sensor endpoints.

1. **Enable Telemetry:** In `Config.h`, set:
   ```c
   #define DEBUG 1
   ```
2. **Flash & Open Serial Monitor:**
   - Open Arduino IDE Serial Monitor at **115200 baud**.
   - Observe the incoming telemetry lines:
     ```text
     WHEEL,0,ACS,...,PEDALS,<accel>,<brake>,<clutch>,MCP,...
     ```
3. **Record Raw Endpoints:**
   - Note the unpressed value and the fully pressed value for each pedal.
   - Example observed values (for ADS1115):
     - Throttle: Resting = `8500`, Fully Pressed = `24000`
     - Brake: Resting = `2000`, Fully Pressed = `20000`
     - Clutch: Resting = `3000`, Fully Pressed = `16000`
   - *(Note: On an ADS1015 12-bit board, these values will be scaled down proportionally by a factor of 16, e.g. ~`530` to ~`1500` for throttle).*
4. **Update Calibration Limits in `Pedals.h`:**
   Open [`main_sketch_GearStickForMargus/Pedals.h`](../main_sketch_GearStickForMargus/Pedals.h) and set your measured values with a small margin of deadzone (to avoid jitter at 0% or 100%):
   ```cpp
   const int16_t ACCELERATOR_MIN_VALUE = 9500;   // Resting value + deadzone
   const int16_t ACCELERATOR_MAX_VALUE = 23000;  // Fully pressed - deadzone
   const int16_t BRAKE_MIN_VALUE       = 2200;
   const int16_t BRAKE_MAX_VALUE       = 19000;
   const int16_t CLUTCH_MIN_VALUE      = 3400;
   const int16_t CLUTCH_MAX_VALUE      = 15500;
   ```
5. **Inverting Pedal Action:**
   If a pedal axis responds backwards (reads 100% when released), simply swap the min and max calibration values:
   ```cpp
   const int16_t ACCELERATOR_MIN_VALUE = 23000; // Inverted
   const int16_t ACCELERATOR_MAX_VALUE = 9500;
   ```
6. **Set Debug back to 0:**
   In `Config.h`, set `#define DEBUG 0` before final gaming use.

---

## 4. Steering Wheel & Encoder Configuration

The steering wheel subsystem in [`Wheel.h`](../main_sketch_GearStickForMargus/Wheel.h) and [`Wheel.cpp`](../main_sketch_GearStickForMargus/Wheel.cpp) manages the quadrature encoder and the virtual limits.

### Encoder Connection
- **Phase A:** Arduino **Pin 0** (`INT2`)
- **Phase B:** Arduino **Pin 1** (`INT3`)
- **VCC:** 5V
- **GND:** Ground

### Adjusting Degrees of Rotation
The rotation angle is determined by the encoder's pulses per revolution (PPR) and the configured min/max limit ticks in [`Wheel.h`](../main_sketch_GearStickForMargus/Wheel.h):

$$\text{Total Rotation Degrees} = \frac{\text{ENCODER\_MAX\_VALUE} - \text{ENCODER\_MIN\_VALUE}}{\text{Encoder Ticks Per Degree}}$$

In the default configuration:
```cpp
#define ENCODER_MIN_VALUE -3600 // Full lock CCW
#define ENCODER_MAX_VALUE  3600 // Full lock CW
```
*(With a 600 PPR encoder in 1-tick/pulse decode mode, 2400 ticks equal 360°, giving $\pm 540^\circ = 1080^\circ$ total rotation).*

To adjust rotation limits:
- For **900° lock-to-lock** with 2400 ticks/rev ($6.666\,\text{ticks}/^\circ$):
  $$\text{Ticks} = \pm 450^\circ \times 6.666 = \pm 3000$$
  ```cpp
  #define ENCODER_MIN_VALUE -3000
  #define ENCODER_MAX_VALUE  3000
  ```

### Soft Virtual Endstops
When you reach full lock, the firmware applies a progressive virtual spring force to push the wheel back:
```cpp
#define MAX_ENDPOINT_FORCES 500       // Peak force applied at soft endstop
#define ENDSTOP_WIDTH_TICKS 100       // Width of endstop deceleration zone (ticks)
```

### Self-Centering Spring
When no game FFB is running, an optional centering spring can pull the wheel to center:
```cpp
#define SELFCENTER_ON       1  // 1 = Enabled, 0 = Disabled
#define SELFCENTER_FORCE   50  // Strength of return spring (out of 250)
#define SELFCENTER_DEADZONE 50 // Center deadzone ticks where no spring force is applied
```

---

## 5. Motor Driver & Current Sensor Tuning

The motor driver is controlled via high-frequency Timer 1 PWM in [`motor.h`](../main_sketch_GearStickForMargus/motor.h) and closed-loop current feedback in [`ACS712Driver.cpp`](../main_sketch_GearStickForMargus/ACS712Driver.cpp).

### Pin Connections
- **Motor PWM Phase A:** Pin 9 (`OC1A`)
- **Motor PWM Phase B:** Pin 10 (`OC1B`)
- **Motor Enable / Gate:** Pin 6
- **ACS712 Current Sense Input:** Pin A5 (ADC8)

### Motor Bias Compensation
DC motors frequently exhibit slight directional friction asymmetry due to mechanical brushes or gearboxes. In `motor.h`:
```cpp
#define CLOCKWISE_BIAS(x) (x + (x>>3) + (x>>4)) // 1.1875x CW multiplier
```
If your motor spins faster in one direction than the other, adjust or remove this macro.

### Current Slew Rate
To prevent abrupt torque spikes that can strip 3D-printed gears or induce electrical ringing:
```cpp
#define SLEW_STEP 100 // Maximum duty cycle change per 1 ms tick
```
Increase `SLEW_STEP` for crisper high-frequency vibrations; decrease it for smoother, quieter wheel operation.

