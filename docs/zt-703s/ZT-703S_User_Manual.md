# ZT-703S Handheld Oscilloscope Multimeter - User Manual

* **Model / Specs:** 25000 Counts | T-RMS | Dual-Channel Oscilloscope Multimeter | Signal Generator
* **Features:** Auto Power Off | 3.5-inch IPS Color Display

---

## Notice & Warranty

* All rights reserved; offenders will be prosecuted.
* Specifications are subject to change without prior notice.

### Limited Warranty and Scope of Rights and Responsibilities
* This product is eligible for a **one-year warranty** from the date of purchase.
* This warranty does not cover blown fuses, damage to general accessories, or damage caused by accidents, negligence, misuse, modifications, pollution, and abnormal operating environments.
* **Note:** If there is a situation of freezing or crashing during use, please restart the device.

---

## Table of Contents

1. [Overview](#1-overview)
2. [Safety Instructions](#2-safety-instructions)
3. [Main Interface & Hardware Layout](#3-main-interface--hardware-layout)
4. [Oscilloscope Mode Main Interface](#4-oscilloscope-mode-main-interface)
5. [Panel Function Keys](#5-panel-function-keys)
6. [MENU Interface](#6-menu-interface)
7. [Oscilloscope Functions Introduction](#7-oscilloscope-functions-introduction)
   - [Probe Check & Safety](#probe-check--safety)
   - [Manual Probe Compensation](#manual-probe-compensation)
   - [Probe Attenuation Setting](#probe-attenuation-setting)
   - [Channel Settings](#channel-settings)
   - [Automatic Setup](#automatic-setup)
   - [Vertical System](#vertical-system)
   - [Horizontal System](#horizontal-system)
   - [Trigger System](#trigger-system)
   - [Numeric Measurement](#numeric-measurement)
   - [XY Display Mode](#xy-display-mode)
   - [Persistence Time, Format & Backlight Time](#persistence-time-format--backlight-time)
   - [Cursor Measurement](#cursor-measurement)
   - [Waveform Storage, Management & System Settings](#waveform-storage-management--system-settings)
   - [Run Modes & Calibration](#run-modes--calibration)
8. [Signal Generator Functions](#8-signal-generator-functions)
   - [Quick Signal Output Settings](#quick-signal-output-settings)
   - [Signal Generator Mode (Extended App)](#signal-generator-mode-extended-app)
9. [Multimeter Functions Introduction](#9-multimeter-functions-introduction)
   - [Multimeter LCD Display Screen](#multimeter-lcd-display-screen)
   - [Multimeter Input Terminals](#multimeter-input-terminals)
   - [Measurement Methods](#measurement-methods)
   - [Multimeter Extended Functions (UART / Upper Computer)](#multimeter-extended-functions)
10. [Maintenance and Care](#10-maintenance-and-care)
    - [Cleaning & Battery Care](#cleaning-the-product)
    - [Battery Replacement](#battery-replacement)
    - [Fuse Replacement](#fuse-replacement)
11. [Technical Specifications](#11-technical-specifications)
    - [General & Mechanical Specifications](#general-technical-specifications)
    - [Multimeter Technical Specifications](#multimeter-technical-specifications)
    - [Oscilloscope Technical Specifications](#oscilloscope-technical-specifications)

---

## 1. Overview

This handheld oscilloscope adopts a dual injection molding process, featuring a beautiful appearance, compact size, convenient portability, and flexible operation. The functional buttons have a clear and intuitive menu interface. The screen utilizes a **3.5-inch IPS full-view color display**, with a multimeter display of up to **25,000 counts**.

This product integrates the functions of an **oscilloscope**, **signal generator**, and **multimeter** into a 3-in-1 device. With superior performance and powerful functionality, it can be used in various measurement scenarios, meeting a wide range of user measurement needs.

---

## 2. Safety Instructions

To avoid possible electric shock, fire hazards, and personal injury, please read the safety precautions before using. Use the product only for its designated purpose, as using it otherwise may compromise the protection it provides.

* Before using the product, check the housing for cracks or plastic damage. Carefully inspect the insulation near the input ports. Follow the instructions in this user manual, use the correct input ports, and set the appropriate range as specified.
* Do not use this product in the presence of explosive gases, vapors, or in humid environments. Keep your fingers behind the protective shield/collar of the test probe.
* Do not touch unused input ports when the product is connected to the circuit under test. Disconnect the test probes and the circuit before changing the test range.
* When the DC voltage under test is higher than **36V**, or the AC voltage is higher than **25V**, it may cause serious harm to the human body; users should be cautious to avoid electric shock.
* Select the correct test range and scale to prevent damage to the instrument or personal injury.
* Do not use this product with the front or rear cover open.
* Low battery voltage may affect the accuracy of test results; please recharge promptly.
* **Grounding Warning:** The ground line between the two channels is shared (the same). During measurements, the ground clip must always be grounded or connected to the same potential.
* The ground wire of the probe is at the same potential as ground. When connecting the USB cable for charging, it is **prohibited** for the ground wire of the probe to touch high voltage, as this may result in product damage or personal injury.
* When using an oscilloscope probe to measure voltage higher than **AC 25V** or **DC 36V**, ensure that the **USB protective cover is securely closed** to prevent human contact with exposed metal parts.

---

## 3. Main Interface & Hardware Layout

### Hardware Ports and Layout
* **Top End:**
  * **CH2 Input** (BNC connector)
  * **CH1 Input** (BNC connector)
* **Right Side:**
  * **USB Interface** (Type-C for charging, PC communication, data transfer)
  * **Signal Generator Output Port**
* **Front Panel:**
  * 3.5-inch IPS Display Screen
  * Function & Control Keypad
* **Bottom End (Multimeter Measurement Input Terminals):**
  * **10A:** Current measurement input terminal (250V MAX, 10A MAX FUSED)
  * **mA:** Current measurement input terminal (250V MAX, 250mA MAX FUSED)
  * **COM:** Common (return) port for all multimeter measurements
  * **VΩHz⯈⯈:** Input port for Voltage, Resistance, Capacitance, Frequency, Continuity, and Diode (600V CAT IV, 1000V CAT III)

---

## 4. Oscilloscope Mode Main Interface

| No. | Element | Description |
|:---:|:---|:---|
| **1** | **Operating Status** | **RUN:** Automatic waveform acquisition state.<br>**WAIT:** Normal trigger mode, waiting for trigger signal.<br>**T.D:** Captured triggered waveform data.<br>**STOP:** Locked waveform, acquisition stopped. |
| **2** | **Time Base Window** | Displays the current time base position within the storage depth. |
| **3** | **Time Base Scale** | Displays the current horizontal time base scale setting. |
| **4** | **Trigger Channel** | Indicates active trigger source (**1** for CH1, **2** for CH2). |
| **5** | **Trigger Edge Mode** | Displays current trigger edge (Rising Edge or Falling Edge). |
| **6** | **Trigger Level** | Displays the set trigger voltage value (e.g., `T:600mV`). |
| **7** | **Battery Indicator** | Displays battery level and charging status. |
| **8** | **Horizontal Trigger Marker** | Indicates the horizontal trigger time position. |
| **9** | **Channel 1 (CH1) Trace** | CH1 waveform displayed in **Yellow**. |
| **10** | **Vertical Trigger Marker** | Indicates the vertical trigger voltage level. |
| **11** | **Channel 2 (CH2) Trace** | CH2 waveform displayed in **Blue**. |
| **12** | **VOL/TIME Menu (F1)** | Adjust channel voltage and time base: Press **F1** to toggle active channel (menu color indicates CH1/CH2). **Up/Down** adjusts voltage amplitude; **Left/Right** adjusts time base scale. |
| **13** | **Waveform Move Menu (F2)** | Short press **F2** to toggle channels. Use **Arrow keys** to shift waveform position. **Long press F2** to reset the waveform to the center position. |
| **14** | **Trigger Cursor Menu (F3)** | Use **Up/Down** to adjust vertical trigger level, and **Left/Right** to adjust horizontal trigger position. |
| **15** | **Measurement Cursor (F4)** | Selects and adjusts cursor measurement axes. |
| **16** | **CH1 Voltage Status** | Displays coupling mode (AC/DC) and vertical scale of CH1. |
| **17** | **CH2 Voltage Status** | Displays coupling mode (AC/DC) and vertical scale of CH2. |
| **18** | **Signal Generator Status** | Displays active output waveform symbol and set frequency (Square, Pulse, Sine, Triangle). |

---

## 5. Panel Function Keys

* **F1 – F4:** Corresponds directly to the four software menu options displayed on the bottom bar of the screen.
* **REL / Power Button:**
  * *Long press (2s):* Power On / Power Off.
  * *Short press (Multimeter mode):* Enters Relative Value (`REL`) measurement mode.
* **AUTO / RANGE:**
  * *Oscilloscope mode:* Automatically acquires measurement waveforms and configures parameters.
  * *Multimeter mode:* Toggles between Auto Range and Manual Range selection.
* **HOLD / SAVE:**
  * *Oscilloscope mode:* Short press toggles `STOP` / `RUN`; Long press saves current waveform data.
  * *Multimeter mode:* Data Hold (freeze/unfreeze reading).
* **MODE:** Toggles between Oscilloscope Mode and Multimeter Mode.
* **Arrow Keys (Up, Down, Left, Right):** Adjust settings, move cursor positions, adjust scales, and navigate menus.
* **MENU:** Toggles the main system function menu on/off (navigated via Left/Right keys).

---

## 6. MENU Interface

Press the **MENU** key to open the on-screen configuration menus:

* **Page 1 (Channel Settings Menu):**
  * `F1` **Channel**: CH1 / CH2
  * `F2` **Enable**: ON / OFF
  * `F3` **Coupling**: DC / AC
  * `F4` **Probe**: X1 / X10
* **Page 2 (Trigger Setup Menu):**
  * `F1` **Trig Mode**: AUTO / NORM / SINGLE
  * `F2` **Edge**: RISING / FALLING
  * `F3` **Trig Source**: CH1 / CH2
  * `F4` **Trig Pos**: 50%
* **Page 3 (Auxiliary Functions Menu 1):**
  * `F1` **Calibrate**: OFF / ENTER
  * `F2` **Default**: OFF / ENTER (Factory Reset)
  * `F3` **USB**: ENTER (Data Save / Mass Storage Mode)
  * `F4` **Language**: ENGLISH / SIMPLIFIED CHINESE
* **Page 4 (Auxiliary Functions Menu 2):**
  * `F1` **Auto Off**: OFF / 1 min / 10 min / 30 min / 60 min / 120 min
  * `F2` **Bk Light**: 30% / 50% / 80% / 100%
  * `F3` **Run mode**: Normal (200MSa/s, 30MHz) / High-speed (280MSa/s, 50MHz)
  * `F4` **Version**: Firmware build version display
* **Page 5 (Extended Functions Menu 1):**
  * `F1` **Display**: Y-T / X-Y
  * `F2` **Persist**: MIN / 500ms / 1S / 10S / Infinite
  * `F3` **Format**: Begin (Clears saved waveform images)
  * `F4` **BL time**: OFF / 30S / 60S / 120S
* **Page 6 (Extended Functions Menu 2):**
  * `F1` **MoreAPPs**: ENTER (Image Viewer, Signal Generator App)
  * `F2` **Cursor**: OFF / Horizontal / Vertical / H+V
  * `F3` **Measure**: ENTER (8 auto measurements selection)
  * `F4` **OutPut**: SETTING (Signal Generator output configuration)

---

## 7. Oscilloscope Functions Introduction

### Probe Check & Safety
* **Safety:** Keep fingers behind the safety ring on the probe body. Do not touch metal parts on the top of the probe when connected to high voltages.
* **Voltage Limits:** Maximum **150V** on **1X** range; Maximum **300V** on **10X** range (DC + AC peak).

### Manual Probe Compensation
When connecting the probe for the first time, perform probe compensation:
1. Power on and connect probe to the Signal Generator Output terminal emitting a **1 kHz square wave**.
2. Press **AUTO** on the panel to display the waveform.
3. Check status:
   * **Compensation Normal:** Sharp square corners.
   * **Overcompensation:** Overshoot spike on rising edge.
   * **Undercompensation:** Rounded slow rising edge.
4. If adjustment is required, insert the non-metallic adjustment tool into the probe trimmer capacitor hole and turn until edges are square.

### Probe Attenuation Setting
Match the probe physical switch with the oscilloscope software menu:
* Probe switch at **X1** $\rightarrow$ Oscilloscope menu set to **X1**.
* Probe switch at **X10** $\rightarrow$ Oscilloscope menu set to **X10**.
> *Note:* At 1X, probe bandwidth is restricted to ~6MHz. Set probe to 10X to access full 50MHz bandwidth.

### Channel Settings
In **Page 1 Menu**:
* **F1 (Channel):** Select CH1 or CH2.
* **F2 (Enable):** Turn the active channel waveform display ON or OFF.
* **F3 (Coupling):** Toggle between **DC** (full signal) and **AC** (blocks DC component).
* **F4 (Probe):** Set probe attenuation multiplier to **X1** or **X10**.

### Automatic Setup
When observing unknown waveforms, press **AUTO**. The oscilloscope automatically identifies waveform type (sine, square, etc.) and optimizes the vertical scale, horizontal time base, and trigger settings.

### Vertical System
* Press **F1 (VOL/TIME)** $\rightarrow$ Use **Up / Down** arrows to adjust vertical scale:
  * **1X Attenuation:** $20\,\text{mV/div} \sim 10\,\text{V/div}$
  * **10X Attenuation:** $200\,\text{mV/div} \sim 100\,\text{V/div}$
* Press **F2 (MOVE)** $\rightarrow$ Use **Up / Down** arrows to shift the vertical waveform position.

### Horizontal System
* Press **F1 (VOL/TIME)** $\rightarrow$ Use **Left / Right** arrows to adjust horizontal time base ($10\,\text{ns/div} \sim 20\,\text{s/div}$).
* Press **F2 (MOVE)** $\rightarrow$ Use **Left / Right** arrows to move waveform horizontally. Long press **MENU** to return horizontal cursor to center ($0\,\text{s}$).
* **Scroll Mode:** Setting the time base to $\ge 200\,\text{ms/div}$ enters Scroll Mode automatically (waveforms flow continuously from left to right, ideal for low-speed signals).

### Trigger System
In **Page 2 Menu**:
* **Trigger Mode (F1):**
  * **AUTO:** Continuously captures and refreshes the waveform.
  * **NORMAL:** Captures and displays waveforms only when trigger condition is met.
  * **SINGLE:** Captures a single triggered waveform, freezes display (`STOP`), and stops acquisition. Press **HOLD** to re-arm.
* **Trigger Edge (F2):** Select **Rising Edge** or **Falling Edge**.
* **Trigger Source (F3):** Select **CH1** or **CH2**.
* **Trigger Position (F4):** Auto-adjusts trigger position to center (50%).
* *Level Adjustment:* From Main Interface, press **F3 (TRIG)** and use Arrow keys to adjust vertical trigger level and horizontal trigger point.

### Numeric Measurement
In **Page 6 Menu**, press **F3 (Measure)**:
* 8 available parameters: Peak-to-Peak ($V_{\text{pp}}$), Maximum ($V_{\text{max}}$), Minimum ($V_{\text{min}}$), Root Mean Square ($V_{\text{rms}}$), Frequency, Duty Cycle, Period, and Frequency Counter.
* CH1 and CH2 can each display up to **4 groups** simultaneously.

### XY Display Mode
In **Page 5 Menu**, press **F1 (Display)** to select **X-Y**:
* Maps CH1 against CH2 for phase difference evaluation and Lissajous pattern analysis.

### Persistence Time, Format & Backlight Time
In **Page 5 Menu**:
* **F2 (Persist):** Select waveform persistence duration: `MIN`, `500ms`, `1S`, `10S`, `Infinite`.
* **F3 (Format):** Formats internal memory and clears all saved waveforms.
* **F4 (BL time):** Configures display backlight timeout: `30S`, `60S`, `120S`, or `OFF`.

### Cursor Measurement
In **Page 6 Menu**, press **F2 (Cursor)**:
* Select **Horizontal Cursor**, **Vertical Cursor**, or **Horizontal + Vertical Cursor**.
* Return to main screen, press **F4 (CURSOR)**, and use arrow keys to adjust cursor positions to read voltage ($\Delta V$) or time ($\Delta T$) deltas.

### Waveform Storage, Management & System Settings
* **Save Waveform:** Long press **HOLD/SAVE** for 2 seconds. Waveforms are saved sequentially into internal memory.
* **Browse on Device:** Page 6 Menu $\rightarrow$ **F1 (MoreAPPs)** $\rightarrow$ select `ImgView` $\rightarrow$ Navigate with arrow keys, press **MENU** to view, **F3** to delete.
* **Access via PC:** Connect to PC via Type-C cable $\rightarrow$ Page 3 Menu $\rightarrow$ **F3 (USB)** $\rightarrow$ Open `pic` folder on the USB Mass Storage device.
* **Language (Page 3 F4):** Toggle Simplified Chinese or English.
* **Auto Shutdown (Page 4 F1):** Select `1m`, `10m`, `30m`, `60m`, `120m`, or `OFF`.
* **Restore Settings (Page 3 F2):** Select **Default**, confirm with **MENU** to reboot to factory defaults.

### Run Modes & Calibration
* **Run Mode (Page 4 F3):**
  * **Normal Mode:** Sampling rate $200\,\text{MSa/s}$, Bandwidth $30\,\text{MHz}$ (power efficient).
  * **High-Speed Mode:** Sampling rate $280\,\text{MSa/s}$, Bandwidth $50\,\text{MHz}$.
* **Backlight Brightness (Page 4 F2):** Select `30%`, `50%`, `80%`, or `100%`.
* **Baseline Calibration (Page 3 F1):**
  1. Disconnect probes and input signals.
  2. Select **Calibrate** and press **MENU**. Do not perform any operations during calibration.

---

## 8. Signal Generator Functions

### Quick Signal Output Settings
In **Page 6 Menu**, press **F4 (OutPut)**:
* **Fields:** Output Waveform Type, Frequency, Amplitude ($V_{\text{pp}}$), Duty Cycle.
* **Adjustment:** Use Up/Down arrows to select field (red border $\rightarrow$ press **MENU** to activate yellow border) $\rightarrow$ use Left/Right arrows to adjust value $\rightarrow$ press **MENU** to save $\rightarrow$ press **F4** to exit.
> *Note:* When outputting pulse, sine, or sawtooth waves, the oscilloscope time base is limited to $\le 100\,\mu\text{s}$.

### Signal Generator Mode (Extended App)
Navigate to **Page 6 Menu** $\rightarrow$ **F1 (MoreAPPs)** $\rightarrow$ select `GEN` $\rightarrow$ press **MENU**:
* **Waveforms:** Sine wave, Square wave, Triangle wave, Half-wave, Full-wave, Sawtooth wave.
* **Controls:**
  * **Up / Down:** Select waveform type.
  * **Left / Right:** Select frequency stepping unit ($1\,\text{Hz}$ / $1\,\text{kHz}$).
  * **F1 / F2:** Increase / Decrease Frequency (`Freq+` / `Freq-`).
  * **F3 / F4:** Increase / Decrease Amplitude / Duty Cycle (`Vpp+` / `Vpp-`).
  * **MENU:** Toggle Output **ON / OFF**.
  * **MODE:** Exit to Oscilloscope mode.

---

## 9. Multimeter Functions Introduction

Press the **MODE** key to enter Multimeter Mode (25,000 Counts display).

### Multimeter LCD Display Screen
1. **HOLD:** Data hold indicator.
2. **Battery level:** Battery charge and status.
3. **Analog pointer:** Dynamic analog arc showing relative scale position.
4. **Symbol display:** Active mode symbol (DC, AC, $\Omega$, diode, buzzer, capacitor).
5. **Negative sign (-)**
6. **Main display:** Primary measurement value (up to 25,000 counts).
7. **Unit symbol:** Active measurement unit ($\text{V}$, $\text{mV}$, $\text{A}$, $\text{mA}$, $\Omega$, $\text{k}\Omega$, $\text{M}\Omega$, $\text{nF}$, $\mu\text{F}$, $\text{mF}$, $\text{Hz}$, etc.).
8. **Testing mode:** `AUTO` (Auto-range) or `MANU` (Manual range via RANGE key).
9. **Max:** Maximum reading.
10. **AVG:** Average reading.
11. **Min / Hz:** Minimum reading (for DCV, $\Omega$, Cap) or Frequency (for ACV, ACA).
12. **F1 Key (Voltage range):** Toggle Voltage mode and AC/DC selection.
13. **F2 Key (Resistance / Continuity / Diode / Capacitance):** Cycles through $\Omega \rightarrow \text{Continuity} \rightarrow \text{Diode} \rightarrow \text{Capacitance}$.
14. **F3 Key (Current range):** Switches to Amperes ($A$) mode.
15. **F4 Key (Millivolt / mA range):** Switches to $\text{mV}$ range (or $\text{mA}$ under current mode).

### Multimeter Input Terminals
* **10A (Red):** Current measurement input $\le 9.999\,\text{A}$ (Fused, 250V MAX, 10A MAX).
* **mA (Red):** Current measurement input $\le 250\,\text{mA}$ (Fused, 250V MAX, 250mA MAX).
* **COM (Black):** Common reference return port.
* **VΩHz⯈⯈ (Red):** Input port for Voltage, Resistance, Capacitance, Frequency, Continuity, and Diode (600V CAT IV, 1000V CAT III).

### Measurement Methods

#### AC and DC Voltage
1. Black probe $\rightarrow$ `COM`, Red probe $\rightarrow$ `VΩHz⯈⯈`.
2. For $<250\,\text{mV}$: press **F4** once for DC mV, twice for AC mV. For $>250\,\text{mV}$: press **F1** once for DC V, twice for AC V.
3. Contact probes to circuit test points and read value.

#### AC and DC Current
1. Black probe $\rightarrow$ `COM`, Red probe $\rightarrow$ `10A` or `mA` terminal.
2. Press **F3** (or **F4** for mA). Press again to toggle DC / AC.
3. Connect probes in series with de-energized test circuit, then restore power.
> *Warning:* Never apply voltage across current terminals.

#### Resistance Measurement
1. Black probe $\rightarrow$ `COM`, Red probe $\rightarrow$ `VΩHz⯈⯈`. Press **F2** to select Resistance ($\Omega$).
2. Ensure test circuit power is OFF and all capacitors are completely discharged. Read resistance value.

#### Continuity Testing
1. Black probe $\rightarrow$ `COM`, Red probe $\rightarrow$ `VΩHz⯈⯈`. Press **F2** in resistance mode to select continuity.
2. Connect across test points; built-in buzzer sounds if circuit is closed/shorted.

#### Diode Testing
1. In continuity mode, press **F2** to enter diode test mode.
2. Red probe to Positive (Anode), Black probe to Negative (Cathode). Read forward voltage drop. `OL` indicates open circuit or reverse polarity.

#### Capacitance Measurement
1. In diode mode, press **F2** once to enter capacitance mode.
2. Fully discharge capacitors before connecting probes. Read capacitance once stabilized.

### Multimeter Extended Functions
Press **MENU** inside Multimeter Mode:
* **F1 (Language):** Chinese / English.
* **F2 (Auto Off):** OFF, 1 min, 10 min, 30 min, 60 min, 120 min.
* **F3 (Bk Light):** 30%, 50%, 80%, 100%.
* **F4 (Uart):** Enable serial output out of the Signal Generator port at **115200 baud** (transmits data 3 times/sec).
> *Warning:* Serial port ground is shared with oscilloscope ground. Do not connect scope probes and UART cable simultaneously to live non-isolated circuits.

---

## 10. Maintenance and Care

### Cleaning the Product
* Clean casing using a damp cloth and mild detergent. Do not use corrosive or solvent-based cleaners. Disconnect all inputs before cleaning.

### Battery Charging & Storage
* Recharge when low battery icon appears using a **5V Type-C adapter** or computer USB port.
* Red power button LED illuminates during charging and turns off/flashes when full.
* For long-term storage (>6 months), charge to 50%–70% and store in a cool, dry place.

### Battery Replacement
* Uses a standard **18650 lithium battery**. Replace if runtime degrades significantly, observing correct polarity.

### Fuse Replacement
1. Disconnect test leads and power off.
2. Unscrew the 4 rear housing screws and remove rear cover.
3. Replace blown fuse with an identical model (**250mA/250V** or **10A/250V** fast-blow). Reattach cover.

---

## 11. Technical Specifications

### General Technical Specifications
* **Display (IPS):** 25000 counts
* **Range:** Automatic / Manual
* **Material:** ABS + TPE
* **Sampling Rate (DMM):** 3 times per second
* **True RMS:** Supported
* **Dimensions:** $177 \times 89 \times 40\,\text{mm}$
* **Weight:** $390\,\text{g}$ ($340\,\text{g}$ without battery)
* **Battery Type:** 18650 battery $\times 1$
* **Warranty:** 1 year
* **Operating Environment:** $0 \sim 40^\circ\text{C}$, $<75\%$ RH
* **Storage Environment:** $-20 \sim 60^\circ\text{C}$, $<80\%$ RH

### Multimeter Technical Specifications

| Function | Range | Resolution | Accuracy |
|:---|:---|:---|:---|
| **DC Voltage (V)** | 2.5000V<br>25.000V<br>250.00V<br>1000.0V | 0.0001V<br>0.001V<br>0.01V<br>0.1V | $\pm(0.05\% + 3)$ |
| **DC Voltage (mV)** | 25.000mV<br>250.00mV | 0.001mV<br>0.01mV | $\pm(0.05\% + 3)$ |
| **AC Voltage (V)** | 2.5000V<br>25.000V<br>250.00V<br>750.0V | 0.0001V<br>0.001V<br>0.01V<br>0.1V | $\pm(0.5\% + 3)$<br>*(40Hz ~ 1kHz)* |
| **AC Voltage (mV)** | 25.000mV<br>250.00mV | 0.001mV<br>0.01mV | $\pm(0.5\% + 3)$<br>*(40Hz ~ 1kHz)* |
| **DC Current (A)** | 2.5000A<br>10.000A | 0.0001A<br>0.001A | $\pm(0.5\% + 3)$ |
| **DC Current (mA)** | 25.000mA<br>250.00mA | 0.001mA<br>0.01mA | $\pm(0.5\% + 3)$ |
| **AC Current (A)** | 2.5000A<br>10.000A | 0.0001A<br>0.001A | $\pm(0.8\% + 3)$<br>*(40Hz ~ 1kHz)* |
| **AC Current (mA)** | 25.000mA<br>250.00mA | 0.001mA<br>0.01mA | $\pm(0.8\% + 3)$<br>*(40Hz ~ 1kHz)* |
| **Resistance ($\Omega$)** | 250.00$\Omega$<br>2.5000k$\Omega$<br>25.000k$\Omega$<br>250.00k$\Omega$<br>2.5000M$\Omega$<br>25.00M$\Omega$<br>250.0M$\Omega$ | 0.01$\Omega$<br>0.0001k$\Omega$<br>0.001k$\Omega$<br>0.01k$\Omega$<br>0.0001M$\Omega$<br>0.01M$\Omega$<br>0.1M$\Omega$ | $\pm(0.5\% + 3)$<br>$\pm(0.2\% + 3)$<br>$\pm(0.2\% + 3)$<br>$\pm(0.2\% + 3)$<br>$\pm(1.0\% + 3)$<br>$\pm(1.0\% + 3)$<br>$\pm(5.0\% + 5)$ |
| **Capacitance** | 9.999nF<br>99.99nF<br>999.9nF<br>9.999$\mu$F<br>99.99$\mu$F<br>999.9$\mu$F<br>9.999mF<br>99.99mF | 0.001nF<br>0.01nF<br>0.1nF<br>0.001$\mu$F<br>0.01$\mu$F<br>0.1$\mu$F<br>0.001mF<br>0.01mF | $\pm(5.0\% + 20)$<br><br>$\pm(2.0\% + 5)$<br><br>$\pm(5.0\% + 5)$ |
| **Frequency** | 9.999Hz<br>99.99Hz<br>999.9Hz<br>9.999kHz<br>99.99kHz | 0.001Hz<br>0.01Hz<br>0.1Hz<br>0.001kHz<br>0.01kHz | $\pm(2.0\% + 2)$<br><br>$\pm(0.1\% + 2)$ |
| **Diode / Continuity** | Forward drop / Buzzer | — | Supported |

### Oscilloscope Technical Specifications

| Parameter | Specification |
|:---|:---|
| **Analog Bandwidth** | $50\,\text{MHz}$ (Dual Channel) |
| **Sampling Rate** | $200\,\text{MSa/s}$ (Normal Mode) / $280\,\text{MSa/s}$ (High-Speed Mode) |
| **Channels** | 2 Channels (CH1, CH2) |
| **Input Coupling** | DC, AC |
| **Input Impedance** | $1\,\text{M}\Omega\text{ @ }16\,\text{pF}$ |
| **Probe Attenuation** | X1, X10 |
| **Max Input Voltage** | X1 range $<150\,\text{V}$, X10 range $<300\,\text{V}$ (DC + AC peak) |
| **Sampling Rate Range** | $1.5\,\text{Sa/s} \sim 280\,\text{MSa/s}$ |
| **Interpolation** | $(\sin x)/x$ |
| **Horizontal Time Base**| $10\,\text{ns/div} \sim 20\,\text{s/div}$ (Accuracy: $20\,\text{ppm}$) |
| **Record Length** | Up to $128\,\text{Kbyte}$ |
| **Vertical Sensitivity** | $20\,\text{mV/div} \sim 10\,\text{V/div}$ |
| **Vertical Offset Range** | $\pm 4\,\text{divisions}$ (positive and negative) |
| **Rise Time** | $<10\,\text{ns}$ |
| **DC Gain Accuracy** | $\pm 3\%$ |
| **Low-Frequency Response**| $>10\,\text{Hz}$ |
| **Auto Measurements** | $V_{\text{pp}}$, $V_{\text{max}}$, $V_{\text{min}}$, $V_{\text{rms}}$, Frequency, Period, Duty Cycle, Frequency Counter |
| **Trigger Modes** | Auto, Normal, Single |
| **Trigger Edges** | Rising edge, Falling edge |
| **Display Modes** | Y-T, X-Y, Roll (Scroll Mode) |
| **Persistence Mode** | Minimum, 500ms, 1S, 10S, Infinite |
| **Signal Generator Waveforms** | Sine wave, Square wave, Sawtooth wave, Half wave, Full wave |
| **Run Modes** | Normal mode ($200\,\text{MSa/s}$), High-speed mode ($280\,\text{MSa/s}$) |
