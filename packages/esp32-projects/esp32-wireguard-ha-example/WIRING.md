# Wiring Guide

This example project primarily focuses on network connectivity (WiFi and WireGuard VPN), so minimal hardware wiring is required. However, this guide provides information for adding optional sensors and controls.

## Basic Setup

### ESP32 Development Board

The basic setup requires only:
- ESP32 development board (ESP32-DevKitC, NodeMCU-32S, or similar)
- USB cable for programming
- Power supply (USB or external 5V)

**No additional wiring is required** for the basic WireGuard and Home Assistant integration!

## Power Considerations

- **USB Power**: 5V via USB port (easiest for development)
- **External Power**: 5V to VIN pin or 3.3V to 3V3 pin
- **Battery Power**: Use 3.7V LiPo with voltage regulator

**Warning**: Never apply more than 3.6V directly to 3V3 pin or more than 6V to VIN pin.

## Optional: I2C Sensors (BME280 Example)

To add environmental sensors like the BME280 (temperature, humidity, pressure):

### BME280 Wiring

| BME280 Pin | ESP32 Pin | Notes |
|------------|-----------|-------|
| VCC        | 3.3V      | Do not use 5V! |
| GND        | GND       | Common ground |
| SDA        | GPIO21    | I2C data (default) |
| SCL        | GPIO22    | I2C clock (default) |

### Configuration

Uncomment these sections in `esp32-wireguard-ha.yaml`:

```yaml
i2c:
  sda: GPIO21
  scl: GPIO22
  scan: true

sensor:
  - platform: bme280
    temperature:
      name: "Temperature"
    pressure:
      name: "Pressure"
    humidity:
      name: "Humidity"
    address: 0x76  # or 0x77 depending on module
    update_interval: 60s
```

## Optional: GPIO Control (Relay/LED)

To control external devices:

### Relay Module Wiring

| Relay Module | ESP32 Pin | Notes |
|--------------|-----------|-------|
| VCC          | 5V (VIN)  | Most relay modules need 5V |
| GND          | GND       | Common ground |
| IN           | GPIO2     | Control signal |

**Note**: If using a 3.3V relay, connect VCC to 3.3V instead.

### Configuration

Uncomment in `esp32-wireguard-ha.yaml`:

```yaml
switch:
  - platform: gpio
    pin: GPIO2
    name: "Relay 1"
    id: relay1
    restore_mode: RESTORE_DEFAULT_OFF
```

## Optional: Physical Button

To add a physical button for triggering actions:

### Button Wiring

| Component | ESP32 Pin | Notes |
|-----------|-----------|-------|
| Button (one side) | GPIO0 | Boot button (built-in on most boards) |
| Button (other side) | GND | Connects to ground when pressed |

**Note**: GPIO0 is the built-in BOOT button on most ESP32 dev boards.

For external button:

| Component | ESP32 Pin | Notes |
|-----------|-----------|-------|
| Button (one side) | GPIO4 | Any available GPIO |
| Button (other side) | GND | Connects to ground when pressed |

### Configuration

Uncomment in `esp32-wireguard-ha.yaml`:

```yaml
binary_sensor:
  - platform: gpio
    pin:
      number: GPIO0
      mode:
        input: true
        pullup: true
      inverted: true
    name: "Button"
    on_press:
      then:
        - switch.toggle: relay1  # If you have a relay
        # Or trigger other actions
```

## GPIO Pin Reference

### Safe to Use (ESP32-DevKitC)

These GPIOs are generally safe for general use:

- GPIO4, GPIO5
- GPIO12, GPIO13, GPIO14, GPIO15
- GPIO16, GPIO17, GPIO18, GPIO19
- GPIO21, GPIO22, GPIO23
- GPIO25, GPIO26, GPIO27
- GPIO32, GPIO33

### Avoid or Use with Caution

- **GPIO0**: Boot mode selection (also BOOT button)
- **GPIO1, GPIO3**: UART TX/RX (used for serial debugging)
- **GPIO6-11**: Connected to flash chip (do not use!)
- **GPIO34-39**: Input only, no pullup/pulldown resistors

### Special Function Pins

- **GPIO21, GPIO22**: Default I2C (SDA, SCL)
- **GPIO18, GPIO19, GPIO23, GPIO5**: Default SPI (SCK, MISO, MOSI, CS)
- **GPIO2**: Often has built-in LED

## Peripheral Examples

### 1. Temperature & Humidity (DHT22)

```
DHT22 Pin 1 (VCC) ──── 3.3V
DHT22 Pin 2 (DATA) ─── GPIO4 (with 4.7kΩ pull-up to 3.3V)
DHT22 Pin 3 (NC) ────── (not connected)
DHT22 Pin 4 (GND) ──── GND
```

### 2. Motion Sensor (PIR)

```
PIR VCC ──── 5V (VIN)
PIR OUT ──── GPIO5
PIR GND ──── GND
```

### 3. LED Indicator

```
LED Anode (+) ──── GPIO2
LED Cathode (-) ─── 220Ω resistor ──── GND
```

### 4. OLED Display (I2C)

```
OLED VCC ──── 3.3V
OLED GND ──── GND
OLED SDA ──── GPIO21
OLED SCL ──── GPIO22
```

## Power Consumption

Typical ESP32 power consumption:

- **Active WiFi**: 160-260mA
- **Active WiFi + WireGuard**: 180-280mA
- **Light sleep**: 0.8mA
- **Deep sleep**: 10-150µA

For battery-powered applications, consider implementing deep sleep modes when not actively communicating.

## Safety Notes

1. **Voltage Levels**: ESP32 GPIOs are 3.3V - do not connect 5V signals directly
2. **Current Limits**: Each GPIO can source/sink ~40mA maximum
3. **Total Current**: Keep total GPIO current under 200mA
4. **Relay Isolation**: Use optocoupler relays for AC loads
5. **ESD Protection**: Handle the ESP32 with care to avoid static damage

## Recommended Development Setup

```
Computer USB Port
      │
      │ USB Cable
      │
      ▼
┌─────────────┐
│   ESP32     │
│  Dev Board  │
└─────────────┘
      │
      │ (Optional) Sensors/Peripherals
      │
      ▼
┌─────────────┐
│ BME280 or   │
│ Other I2C   │
│ Sensor      │
└─────────────┘
```

## Enclosure Considerations

When deploying in an enclosure:

1. **Ventilation**: Ensure airflow for temperature sensors
2. **Antenna placement**: Keep WiFi antenna away from metal
3. **Access**: Leave USB port accessible for debugging
4. **LED visibility**: Position status LEDs where visible
5. **Button access**: Make physical buttons easily reachable

## Breadboard Layout Example

For prototyping with BME280 and relay:

```
     ESP32           BME280         Relay
      │                │              │
  ┌───┴───┐        ┌──┴──┐       ┌───┴────┐
  │ 3.3V  ├────────┤ VCC │       │        │
  │ GND   ├────┬───┤ GND │───────┤ GND    │
  │ GPIO21├────┼───┤ SDA │       │        │
  │ GPIO22├────┼───┤ SCL │       │        │
  │ GPIO2 ├────┼───┼─────┼───────┤ IN     │
  │ 5V    ├────┼───┼─────┼───────┤ VCC    │
  └───────┘    │   └─────┘       └────────┘
               │
            Common
            Ground Bus
```

## Testing

After wiring:

1. **Visual inspection**: Check all connections
2. **Continuity test**: Verify ground connections
3. **Power test**: Measure voltage at VCC pins (should be 3.3V)
4. **Upload firmware**: Flash the basic configuration
5. **Check logs**: Monitor for any I2C or GPIO errors

## Troubleshooting

**Sensor not detected:**
- Check I2C address (use `scan: true` in i2c config)
- Verify wiring (SDA/SCL not swapped)
- Check pull-up resistors (usually built-in to modules)

**Relay not switching:**
- Verify control voltage (should be 3.3V when on)
- Check relay module voltage requirements
- Test GPIO output with LED first

**ESP32 not booting:**
- Check GPIO0 is not grounded at boot (except for flashing)
- Verify power supply can provide sufficient current
- Disconnect peripherals and test with bare ESP32

## Next Steps

1. Start with the basic configuration (no extra hardware)
2. Verify WireGuard and Home Assistant connectivity
3. Add one sensor/peripheral at a time
4. Test each addition before proceeding
5. Document your final wiring for future reference
