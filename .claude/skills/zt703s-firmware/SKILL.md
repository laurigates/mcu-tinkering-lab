---
name: zt703s-firmware
description: The ZOYI ZT-703S handheld oscilloscope-multimeter's USB-C port - update the firmware from a .ZTK file over its mass-storage bootloader and recover from a failed update, and copy saved waveform images to the Mac. Use when a new ZT-703S firmware is downloaded, the meter's version needs checking, an update fails with "insufficient memory" or a blank screen, or saved waveforms are wanted on the computer.
argument-hint: <path to ZOYI-703S-FW-*.ZTK>
allowed-tools: Read, Bash
---

# ZOYI ZT-703S Firmware Update and USB-C

No flashing tool is needed. In upgrade mode the meter mounts on the Mac as a
USB drive, and copying the `.ZTK` file onto it starts the update.

## Before starting

- **Check the file.** A ZT-703S image is named `ZOYI-703S-FW-V<x.y.z>.ZTK`, and
  bytes 8–10 of its header read `OSC`:
  `xxd -l 16 <file>` shows `....OSC7EF91...` for V1.6.27. A file without the
  `OSC` tag is not a firmware image for this meter.
- **Read the current version.** MENU, press right three times; the version is
  the rightmost entry. F4 shows the boot and app versions
  (`Boot: V1.0.13 APP: V1.0.9` on an early unit, per EEVblog).
- **Charge the battery.** The update must not lose power partway through.
- **Factory reset first** if a previous attempt failed with "insufficient
  memory": third menu page, F2 (ZOYI UK support post).

## Update

1. Connect the meter to the Mac over USB-C. With the meter off, hold **F1** and
   press **power** to enter upgrade mode (ZOYI UK). The drive mounts as
   `/Volumes/FLASH`.
2. Copy the image into the drive's `firmware/` folder, without Finder's `._`
   resource-fork file:

   ```sh
   cp -X <file>.ZTK /Volumes/FLASH/firmware/
   ```

   On 2026-11-23 the V1.6.27 update started by itself as soon as the copy
   finished; no eject or button press was needed.
3. Leave the cable connected and the meter on until it reboots by itself.
4. Check the version again (see above). It must show the new app version.

The ZOYI UK post describes copying to the drive's root and ejecting to start
the install, and clearing the file's read-only flag on Windows. Neither was
needed on macOS with the `firmware/` folder. If a copy to `firmware/` does
nothing, try the root and then `diskutil eject /Volumes/FLASH`.

## If it fails

The bootloader is separate from the application, so a failed or interrupted
update can be retried: enter upgrade mode again (F1 + power) and copy the
file again. If the copy is refused for space, factory reset and retry. If the
drive does not mount at all, try another cable (a charge-only USB-C cable
powers the meter but carries no data) and check `diskutil list`.

## Copy saved waveforms

The USB-C port's other documented use (manual, printed p.18):

1. Hold **Save** for 2 seconds. The meter stores the current waveform as a
   numbered image in internal memory.
2. Third menu page, **F3**: data save mode.
3. Connect a USB-C data cable. A "USB Disk" drive mounts; the images are in
   its `pic` folder. Copy them off with `cp -X`.

On the meter, the sixth menu page, F1 (extended applications) shows the saved
images, and F3 there deletes one.

Not documented, and not yet checked on this meter: whether that drive is the
same `/Volumes/FLASH` the bootloader mounts, the image format, and whether the
meter enumerates over USB at all outside these two modes. Compare
`diskutil list` in normal mode, data save mode and upgrade mode, and record
the result here. The meter's serial output is not on USB-C; it is on the
signal-generator jack (`zt703s-uart` skill).

## Sources

- ZOYI UK, "zoyi zt 703s firmware update insufficient mem":
  <https://zoyi.co.uk/smartblog/zoyi-zt-703s-firmware-update-insufficient-mem.html>.
  Not an official ZOYI manual; ZOYI's upgrade video is linked from it.
- ZOYI ZT-703S owner's manual, "How to Save and Review Measurement"
  (printed p.18): <https://www.manualslib.com/manual/3351110/Zoyi-Zt-703s.html?page=22>
- EEVblog ZT-70xS thread, page 6, replies #126–#127 (version menu):
  <https://www.eevblog.com/forum/testgear/new-zoyi-multimeteroscilloscopes-zt-70xs-up-to-50mhz250msps-(nov-2022)/125/>
- The `firmware/` folder and the automatic start: observed by Lauri on
  2026-11-23 with V1.6.27.
