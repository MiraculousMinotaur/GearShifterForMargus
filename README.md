# GearShifterForMargus

Custom DIY Sim-Racing and Sim-Trucking peripheral controller firmware built for the **Arduino Leonardo** (ATmega32U4).

This project integrates three essential sim-driving hardware components into a single USB HID composite gaming device:
1. **Force Feedback Steering Wheel** with optical/magnetic quadrature encoder and closed-loop motor current control via ACS712.
2. **3 Analog Pedals** (Throttle, Brake, Clutch) read with external I2C ADC (ADS1115 16-bit or ADS1015 12-bit).
3. **Multi-mode H-Pattern Shifter** (6 forward gears, reverse, range switch, and splitter toggles) powered by a 16-bit I2C GPIO expander (MCP23017).

> [!IMPORTANT]
> **Windows-Only Force Feedback Notice:**
> Force Feedback (FFB) currently works **only under Windows**. The underlying `ArduinoJoystickWithFFBLibrary` relies on the DirectInput USB HID PID (Physical Interface Device) drivers provided natively by Windows. Under Linux and macOS, the device will enumerate and function as a standard joystick/wheel with pedals and shifter, but FFB effect commands are not supported out of the box.

---

## Features

- **Arduino Leonardo / ATmega32U4 Native USB HID:**
  - Emulates a DirectX / DirectInput gaming controller without external drivers.
  - High-speed HID report updates.
- **Closed-Loop Force Feedback (FFB):**
  - High-frequency 1 kHz PID current loop executing inside Hardware Timer 3 interrupt (`ISR(TIMER3_COMPA_vect)`).
  - ACS712 current sensing with internal 10-bit free-running ADC and hardware filtering to prevent motor over-current and improve realism.
  - Slew-rate limiting and soft virtual endstops with configurable spring constants.
  - Configurable self-centering spring with adjustable deadzone when game FFB is idle.
- **Smooth Analog Pedals:**
  - External **ADS1115** (16-bit) or **ADS1015** (12-bit) I2C ADC.
  - Non-blocking state-machine channel cycling preventing bus contention.
  - Dedicated independent axes: Throttle (`Rx`), Brake (`Ry`), Clutch (`Z`).
- **Realistic Truck / Racing Shifter:**
  - **MCP23017** 16-channel I2C expander for all switch inputs.
  - Reads 6-way H-gate, Reverse switch, Range switch (High/Low), and Splitter toggles in a single fast I2C register transaction (`readGPIOAB`).
  - Realistic neutral-state detection and switch debouncing.
- **I2C Power Control:**
  - Dedicated hardware enable pin (Pin D8) for external sensor bus power stabilization during boot.
- **Non-blocking Telemetry Debugging:**
  - Optional CSV streaming over USB Serial (`DEBUG 1` in `Config.h`) for sensor calibration and PID tuning without stalling the control loop.

---

## Hardware Pinout Reference

### Arduino Leonardo (ATmega32U4)
| Pin | Function | Subsystem | Description |
|---|---|---|---|
| **Pin 0 (INT2 / RX)** | Phase A | Wheel Encoder | Hardware interrupt on pin change |
| **Pin 1 (INT3 / TX)** | Phase B | Wheel Encoder | Hardware interrupt on pin change |
| **Pin 2 (SDA)** | I2C Data | Pedals & Shifter | Connected to ADS1115 & MCP23017 SDA |
| **Pin 3 (SCL)** | I2C Clock | Pedals & Shifter | Connected to ADS1115 & MCP23017 SCL (400 kHz) |
| **Pin 4** | Sensor Power / Enable | ACS712 | Current sensor logic power / enable |
| **Pin 6** | Motor Enable | Motor Driver | H-bridge enable / PWM gate |
| **Pin 8** | I2C Power Switch | Power Control | MOSFET / transistor switch to power I2C devices |
| **Pin 9** | Motor Phase A | Motor Driver | Timer 1 high-frequency PWM output (~21 kHz) |
| **Pin 10** | Motor Phase B | Motor Driver | Timer 1 high-frequency PWM output (~21 kHz) |
| **Pin A5 (ADC8)** | Sense Voltage | ACS712 | Free-running ADC analog reading for current sensing |

### ADS1115 / ADS1015 (Pedals I2C ADC @ 0x48)
| Channel | Function | Default Range (ADS1115) | Default Range (ADS1015) |
|---|---|---|---|
| **A0** | Brake Pedal | `2000` – `20000` | `125` – `1250` |
| **A1** | Accelerator Pedal | `8500` – `24000` | `530` – `1500` |
| **A2** | Clutch Pedal | `3000` – `16000` | `190` – `1000` |

### MCP23017 (Shifter I2C GPIO Expander @ 0x20)
| MCP Pin | Name | Logical Function | Default Joystick Button |
|---|---|---|---|
| **GPB2 (Pin 10)** | `SIX_WAY_PIN_1` | Gear 1 | Button 0 (`NORMAL_1`) |
| **GPB3 (Pin 11)** | `SIX_WAY_PIN_2` | Gear 2 | Button 1 (`NORMAL_2`) |
| **GPB4 (Pin 12)** | `SIX_WAY_PIN_3` | Gear 3 | Button 2 (`NORMAL_3`) |
| **GPB5 (Pin 13)** | `SIX_WAY_PIN_4` | Gear 4 | Button 3 (`NORMAL_4`) |
| **GPB6 (Pin 14)** | `SIX_WAY_PIN_5` | Gear 5 | Button 4 (`NORMAL_5`) |
| **GPB7 (Pin 15)** | `SIX_WAY_PIN_6` | Gear 6 | Button 5 (`NORMAL_6`) |
| **GPA0 (Pin 0)** | `MODE_PIN_1` | Reverse Mode / Switch | Button 6 (`REVERSE`) |
| **GPA1 (Pin 1)** | `MODE_PIN_2` | Range Switch (High/Low) | Button 7 (`HIGH_RANGE`) |
| **GPA6 (Pin 6)** | `LOWER_IMPULSE_PIN` | Splitter Toggle (Lower) | Button 8 (`IMPULSE_1`) |
| **GPA7 (Pin 7)** | `HIGHER_IMPULSE_PIN` | Splitter Toggle (Higher) | Button 9 (`IMPULSE_2`) |

---

## Quick Start & Build Instructions

### 1. Clone with Submodules
This repository relies on several bundled libraries (as submodules in `lib/`):
```bash
git clone --recurse-submodules https://github.com/MiraculousMinotaur/GearShifterForMargus.git
```
If you already cloned without submodules:
```bash
cd GearShifterForMargus
git submodule update --init --recursive
```

### 2. Install Required Libraries into Arduino Sketchbook
The Arduino IDE expects libraries to reside in your personal sketchbook folder (`Documents/Arduino/libraries`). You can create directory junctions on Windows so that git submodule updates are instantly visible to the IDE:

```powershell
New-Item -ItemType Directory -Force -Path "$HOME\Documents\Arduino\libraries"
cmd /c mklink /J "$HOME\Documents\Arduino\libraries\ArduinoJoystickWithFFBLibrary" "C:\Repos\GearShifterForMargus\lib\ArduinoJoystickWithFFBLibrary"
cmd /c mklink /J "$HOME\Documents\Arduino\libraries\Adafruit_ADS1X15" "C:\Repos\GearShifterForMargus\lib\Adafruit_ADS1X15"
cmd /c mklink /J "$HOME\Documents\Arduino\libraries\Adafruit_BusIO" "C:\Repos\GearShifterForMargus\lib\Adafruit_BusIO"
cmd /c mklink /J "$HOME\Documents\Arduino\libraries\Adafruit_MCP23017" "C:\Repos\GearShifterForMargus\lib\Adafruit_MCP23017"
```

### 3. Compile and Upload
#### Using Arduino IDE (2.x):
1. Open `main_sketch_GearStickForMargus/main_sketch_GearStickForMargus.ino`.
2. Select **Tools > Board > Arduino AVR Boards > Arduino Leonardo**.
3. Select your device's COM port under **Tools > Port**.
4. Click **Upload** (`Ctrl + U`).

#### Using Arduino CLI:
```bash
arduino-cli compile --fqbn arduino:avr:leonardo main_sketch_GearStickForMargus --libraries lib --upload -p COMx
```

---

## Configuration (`Config.h`)

All modular subsystems and feature flags are centralized in [`main_sketch_GearStickForMargus/Config.h`](main_sketch_GearStickForMargus/Config.h):

```c
#define FIRMWARE_VERSION "1.0.0"

// Subsystem enable flags
#define DEBUG 0    // Set to 1 to enable serial CSV telemetry (500ms broadcast)
#define PEDALS 1   // Enable ADS1115 pedals (Throttle, Brake, Clutch)
#define GEARS 1    // Enable MCP23017 H-shifter & toggles
#define WHEEL 1    // Enable steering wheel encoder & limits
#define FFB 1      // Enable closed-loop force feedback motor control
```

---

## Detailed Documentation Guides

For comprehensive technical explanations and walkthroughs, refer to the documents in the `docs/` directory:

- 📘 [**Firmware Architecture & Timing Guide**](docs/ARCHITECTURE.md):
  Detailed breakdown of the dual-rate loop design, Timer 3 ISR, non-blocking I2C interleaving, ACS712 closed-loop current PID, and the telemetry system.
- 🛠️ [**Hardware Configuration & Customization Guide**](docs/HARDWARE_CONFIGURATION.md):
  Instructions on adapting the firmware for different button layouts, adding more switches or an 8-speed gate, rewiring active-high/active-low inputs, tuning wheel encoder limits, and recalibrating pedal potentiometers.
- 🚛 [**Euro Truck Simulator 2 (ETS2 / ATS) Setup Guide**](docs/ETS2_SETUP.md):
  Step-by-step configuration for ETS2/ATS Range-Splitter transmissions, button assignments, switch mode behavior, Windows Force Feedback setup, and installation instructions for the included custom [`docs/gearbox_layout_MARGUS_12_2.sii`](docs/gearbox_layout_MARGUS_12_2.sii) gearbox layout.

---

## Release Notes

### Version 1.0.0
- **Removed Serial Debug Command Input:** Stripped out legacy serial input parser (`processCommand`) and input buffers that caused communication delays and overloaded the Leonardo's single CPU core.
- **Streamlined Telemetry Debugging:** Maintained lightweight, non-blocking 500ms CSV telemetry transmission when `DEBUG 1` is configured.
- **Optimized Main Loop Scheduling:** 3ms slow scheduler for USB PID handling and floating-point effect generation.
- **I2C Bus Interleaving:** Solved I2C bus collision between ADS1115 continuous analog sampling and MCP23017 GPIO polling.
- **Full Support for Range & Splitter:** Built-in debounce and neutral-gate verification for reliable truck shifting.

