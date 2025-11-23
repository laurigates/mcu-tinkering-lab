# Audiobook Player

An RFID-based audiobook player that integrates with Home Assistant. Scan picture cards with RFID tags to play different audiobooks, with physical play/pause buttons.

## Hardware

- **WEMOS WiFi & Bluetooth Battery ESP32** (with 18650 battery holder)
- **RC522 RFID Reader Module**
- **Green button** (Play)
- **Red button** (Pause)
- **Tilt switch** (SW-200D or similar) - for wake from deep sleep
- **RFID tags** attached to picture cards
- **Optional:** 18650 battery for portable operation
  - Always-on: ~17 hours runtime
  - With deep sleep: ~29 days runtime

## Wiring

### RC522 RFID Reader (SPI)

| RC522 Pin | ESP32 Pin | GPIO |
|-----------|-----------|------|
| SDA (CS)  | GPIO17    | GPIO17 |
| SCK       | GPIO18    | GPIO18 |
| MOSI      | GPIO23    | GPIO23 |
| MISO      | GPIO19    | GPIO19 |
| IRQ       | *not used* | -    |
| GND       | GND       | -    |
| RST       | RST or 3.3V | -    |
| 3.3V      | 3.3V      | -    |

**Note:** GPIO17 is used instead of GPIO5 to avoid boot issues with strapping pins.

### Buttons

| Button | ESP32 Pin | GPIO | Notes |
|--------|-----------|------|-------|
| Green (Play)  | GPIO32 | GPIO32 | Connect to GND when pressed |
| Red (Pause)   | GPIO33 | GPIO33 | Connect to GND when pressed |

**Note:** Buttons use internal pull-up resistors, so connect one side to the GPIO pin and the other to GND.

### Tilt Switch (Power Management)

| Component | ESP32 Pin | GPIO | Notes |
|-----------|-----------|------|-------|
| Tilt Switch | GPIO27 | GPIO27 | Wakes device from deep sleep when tilted |

**How it works:**
- Device lying flat → Deep sleep (~10µA)
- Pick up/tilt device → Wakes up and connects to WiFi (~3-5 seconds)
- After 2 minutes idle → Returns to deep sleep automatically

### Status LED

The built-in LED (GPIO16) is used as a status indicator.

## Setup

### 1. Install ESPHome

```bash
pip install esphome
```

### 2. Configure Secrets

```bash
cp secrets.yaml.example secrets.yaml
# Edit secrets.yaml with your WiFi credentials
```

### 3. Compile and Upload

First upload via USB:

```bash
esphome run audiobook-player.yaml
```

After initial setup, you can update wirelessly:

```bash
esphome run audiobook-player.yaml --device audiobook-player.local
```

### 4. Read RFID Tag UIDs

After uploading, watch the logs to see tag UIDs:

```bash
esphome logs audiobook-player.yaml
```

Scan your RFID tags and note the UIDs that appear in the logs (format: `XX-XX-XX-XX`).

## Home Assistant Integration

### Automatic Discovery

The device will automatically appear in Home Assistant via the ESPHome integration.

### Automation Example

Create an automation in Home Assistant to map RFID tags to audiobooks:

```yaml
# configuration.yaml or automations.yaml

automation:
  # Handle RFID tag scans
  - alias: "Audiobook - Play on Tag Scan"
    trigger:
      - platform: event
        event_type: tag_scanned
        event_data:
          device_id: !secret audiobook_player_device_id
    action:
      - choose:
          # Harry Potter and the Philosopher's Stone
          - conditions:
              - condition: template
                value_template: "{{ trigger.event.data.tag_id == '74-10-37-94' }}"
            sequence:
              - service: media_player.play_media
                target:
                  entity_id: media_player.living_room_speaker
                data:
                  media_content_id: "spotify:album:your_album_id"
                  media_content_type: music

          # The Gruffalo
          - conditions:
              - condition: template
                value_template: "{{ trigger.event.data.tag_id == 'AA-BB-CC-DD' }}"
            sequence:
              - service: media_player.play_media
                target:
                  entity_id: media_player.living_room_speaker
                data:
                  media_content_id: "spotify:album:another_album_id"
                  media_content_type: music

  # Handle play button
  - alias: "Audiobook - Play Button"
    trigger:
      - platform: event
        event_type: esphome.audiobook_control
        event_data:
          action: play
    action:
      - service: media_player.media_play
        target:
          entity_id: media_player.living_room_speaker

  # Handle pause button
  - alias: "Audiobook - Pause Button"
    trigger:
      - platform: event
        event_type: esphome.audiobook_control
        event_data:
          action: pause
    action:
      - service: media_player.media_pause
        target:
          entity_id: media_player.living_room_speaker
```

### Finding Your Device ID

1. Go to **Settings** → **Devices & Services**
2. Find "Audiobook Player" under ESPHome
3. Click on it and copy the device ID from the URL

### Alternative: Simple Tag to Playlist Mapping

If you prefer a simpler approach using input_select:

```yaml
# helpers.yaml
input_select:
  last_scanned_audiobook:
    name: "Last Scanned Audiobook"
    options:
      - "None"
      - "Harry Potter 1"
      - "The Gruffalo"
      - "Room on the Broom"

# automations.yaml
automation:
  - alias: "Map RFID to Audiobook"
    trigger:
      - platform: event
        event_type: tag_scanned
    action:
      - choose:
          - conditions: "{{ trigger.event.data.tag_id == '74-10-37-94' }}"
            sequence:
              - service: input_select.select_option
                target:
                  entity_id: input_select.last_scanned_audiobook
                data:
                  option: "Harry Potter 1"
              - service: media_player.play_media
                data:
                  media_content_id: "your_media_id_here"
                  media_content_type: music
                target:
                  entity_id: media_player.living_room_speaker
```

## Troubleshooting

### RFID Reader Not Working

1. Check wiring - especially SPI connections
2. Verify 3.3V power supply (not 5V!)
3. Try reducing `update_interval` to 1s
4. Check logs for communication errors

### Buttons Not Responding

1. Verify buttons are connected to correct GPIO pins
2. Check that buttons connect GPIO to GND when pressed
3. Increase debounce time if buttons trigger multiple times

### WiFi Connection Issues

1. Check credentials in `secrets.yaml`
2. Device will create a fallback AP named "Audiobook-Player" if WiFi fails
3. Check router logs for connection attempts

## Next Steps

1. **Attach RFID tags to picture cards** - Use stickers or tape
2. **Scan all tags** and record their UIDs
3. **Create Home Assistant automations** mapping tags to audiobooks
4. **Design an enclosure** to hold the ESP32, RC522, and buttons
5. **Add more features**:
   - Volume control buttons
   - Next/Previous track buttons
   - Sleep timer
   - LED indicators for different states
   - Battery level monitoring (using ESP32's built-in ADC)

## Media Player Options

This project works with any Home Assistant media player:

- **Spotify** (via Spotify integration)
- **Local files** (via Music Player Daemon)
- **YouTube Music** (via YouTube Music integration)
- **Plex** (via Plex integration)
- **Sonos, Google Home, Amazon Echo** (native integrations)

Configure your preferred media player in Home Assistant and update the automations accordingly.
