# Research New Microcontroller Boards

Research the latest microcontroller board announcements and generate concise, technical reports.

## Focus Areas
- ESP32 family (priority)
- STM32 family (priority)
- Adjacent interesting platforms (Seeed XIAO, Adafruit Feather, etc.)

## Reference Boards (for comparisons)
- **ESP32**: ESP32-C6, ESP32-S3, Heltec WiFi LoRa 32
- **STM32**: Blue Pill (STM32F103), Black Pill (STM32F411)

## Project Context
The user's projects focus on:
- **Home automation** - Home Assistant/ESPHome integration, WiFi, MQTT, battery operation
- **Kid gadgets** - RFID, audio (I2S, piezo), safe/enclosed designs, simple interfaces
- **Robotics** - Camera (OV2640), motors, LoRa, servo control, AI/LLM integration
- **Key peripherals**: I2C (OLED, PCA9685), SPI (RFID RC522), UART, cameras, audio

## Sources to Check

### Official Manufacturers
1. **Espressif**: https://www.espressif.com/en/company/newsroom/news and https://developer.espressif.com/
2. **STMicroelectronics**: https://newsroom.st.com/
3. **Seeed Studio**: https://www.seeedstudio.com/blog/
4. **Adafruit**: https://blog.adafruit.com/
5. **SparkFun**: https://www.sparkfun.com/news

### Quality Outlets
1. **CNX Software** (priority): https://www.cnx-software.com/
2. **Hackaday**: https://hackaday.com/
3. **Hackster.io**: https://www.hackster.io/
4. **All About Circuits**: https://www.allaboutcircuits.com/

## Instructions

1. **Search for recent announcements** (last 3-6 months):
   - Check manufacturer newsrooms and blogs
   - Check CNX Software and quality outlets
   - Focus on boards in mass production or near-term availability

2. **Filter for relevance**:
   - Prioritize boards suitable for home automation, robotics, or kid-friendly gadgets
   - Skip pure development boards without practical maker appeal
   - Skip boards with unclear availability or pricing

3. **Generate reports** using this template for 3-5 most interesting boards:

```markdown
## [Board Name]
**Manufacturer:** | **Release/Status:** | **Price:**

### Quick Verdict
[1-2 sentences: Why is this interesting for home automation/kid gadgets/robotics?]

### Key Specs
- **Core:** [CPU, clock, cores]
- **Memory:** [RAM/Flash/PSRAM]
- **Wireless:** [WiFi 6/BLE/LoRa/Thread/Matter/Zigbee]
- **GPIO:** [count, ADC channels, PWM, touch]
- **Interfaces:** [I2C/SPI/UART/I2S/USB]
- **Camera:** [support yes/no, which sensors]
- **Audio:** [I2S, built-in DAC/ADC]
- **Power:** [voltage range, deep sleep current, battery support/charging]
- **Form Factor:** [size, mounting holes, unique features]

### Comparison to Reference Boards
[Brief comparison highlighting improvements or trade-offs vs ESP32-C6/S3 or STM32 Blue/Black Pill]
Example: "Similar to ESP32-C6 but adds WiFi 6E and NPU for 3x ML performance"

### Relevance to Your Projects
- **Home Automation:** [ESPHome support, HA integration, battery life, wireless protocols]
- **Kid Gadgets:** [audio capabilities, safety features, simplicity, enclosure options]
- **Robotics:** [camera support, motor control, sensor ecosystem, real-time performance]

### Notable Features / Why It Stands Out
[What's genuinely NEW - not marketing fluff. Focus on technical advantages]

### Potential Gotchas
[Library/toolchain maturity, availability issues, known bugs, price concerns]
```

4. **Format requirements**:
   - Be CONCISE - avoid marketing prose and filler
   - Focus on technical specs relevant to the project types
   - Compare to reference boards for quick context
   - Flag any concerns (availability, price, toolchain issues)
   - If a board isn't interesting, don't include it

5. **Summary section** at the end:
   - "Worth getting now" - boards to consider immediately
   - "Watch list" - interesting but wait for availability/toolchain maturity
   - "Pass" - hyped but not relevant

## Output Format
Present reports in a clean markdown format. Start with a brief summary of what you found, then provide individual board reports, then the final recommendation summary.
