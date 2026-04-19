# ThinkPack — Troubleshooting

If something is not behaving the way you expect, work through this list
in order. Each problem has the most likely cause first.

## The boxes don't see each other

**Symptom:** Two boxes are both switched on in the same room, but their
LEDs are not pulsing in sync and they don't react to each other.

- **Cause:** One of the boxes still has an old firmware that picked a
  different WiFi channel (1 vs 6). All ThinkPack boxes must be on the
  same channel to mesh.
  **Fix:** Update both boxes to the latest firmware via the web flasher.

- **Cause:** You are more than a few metres apart with something solid
  in between (fridge, stone wall).
  **Fix:** Move the boxes closer together. ESP-NOW works through normal
  walls and doors up to ~10 m line-of-sight, but metal appliances block
  it.

- **Cause:** One of the boxes is in critical battery state. The beacon
  rate drops to one every 5 seconds to save power and discovery takes
  longer.
  **Fix:** Charge the box. Pulsing red LED confirms this is the cause.

## No sound from Boombox

**Symptom:** Boombox is on, the LED is animating, but no sound.

- **Cause:** Volume pot (top knob) is all the way down.
  **Fix:** Twist it clockwise until you hear a tone.

- **Cause:** Melody is set to SILENCE (the fourth pattern).
  **Fix:** Short-press the button once; the pattern advances to MARCH
  and a drum tick should return immediately.

- **Cause:** The piezo element has come unseated (it is glued behind the
  front panel).
  **Fix:** Open the case and re-seat the piezo against the flex PCB pad.

## Glowbug LEDs are dim or flickering

**Symptom:** The ring looks gray/brown instead of white; colours look
washed out.

- **Cause:** Low battery. The LED driver cannot supply full current below
  ~3.4 V and the red component saturates before blue/green do.
  **Fix:** Charge the battery. Finding that the symptoms improve during
  USB charging confirms this.

## Finderbox doesn't detect my tag

**Symptom:** I tap a tag on Finderbox and nothing happens.

- **Cause:** The tag is an incompatible family (NTAG 203 without user
  memory, or a 125 kHz HID proximity card — Finderbox only reads
  13.56 MHz MIFARE/NTAG).
  **Fix:** Try a different tag. If you can program tags from your
  phone, it almost certainly works with Finderbox.

- **Cause:** The tag is too far from the reader. The RC522 has a very
  short range — 2-3 cm at most.
  **Fix:** Put the tag flat against the top surface, centred.

- **Cause:** The tag has not been registered.
  **Fix:** See [Using NFC tags](nfc-tags.md) for the registration
  procedure.

## OTA update stuck at X%

**Symptom:** An OTA update starts, shows progress for a while, then
stops and never completes.

- **Cause:** The receiving box dropped out of range during the transfer.
  OTA uses ESP-NOW fragments; if even one fragment is missed past the
  retry window, the box aborts.
  **Fix:** Bring all boxes within about 1 m of the broadcaster, power-
  cycle the receiver, and re-trigger the OTA. Do not move any box until
  the progress LED shows green/solid.

- **Cause:** The receiver's battery was low and it rebooted during
  verification.
  **Fix:** Charge the receiver first, then retry.

## Brainbox can't connect to my WiFi

**Symptom:** Brainbox's OLED shows "WiFi: ---" forever.

- **Cause:** The network name or password is wrong in NVS. This is most
  common right after a factory reset.
  **Fix:** Connect Brainbox over USB, open the [web flasher](../flasher/index.html),
  click "Configure", and enter your credentials. Brainbox uses the
  Improv WiFi protocol for browser-based setup.

- **Cause:** Your WiFi is 5 GHz only. ESP32 only speaks 2.4 GHz.
  **Fix:** Enable the 2.4 GHz band on your router, or create a guest
  2.4 GHz network for the boxes.

## Nothing I've tried works

Open an issue at
<https://github.com/laurigates/mcu-tinkering-lab/issues> with the
symptom, what you tried, and (if possible) the serial log — hold the box
over a laptop running the web flasher and the "Logs & Console" button
exposes its output.
