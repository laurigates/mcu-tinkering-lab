# ThinkPack Chatterbox

Touch-driven audio record / playback box in the ThinkPack modular-toy mesh.
Runs on an ESP32-S3 SuperMini with:

- **INMP441** I2S MEMS microphone (capture)
- **MAX98357A** I2S DAC + speaker (playback)
- A single native **capacitive touch pad** as the record trigger

Standalone behaviour is press-to-record, release-to-play.  When the mesh
sees other boxes, the Chatterbox acts as the leader of an **Echo Chamber**:
each peer plays the captured clip back at a different pitch, producing a
chorus of voices.  See [`docs/requirements/PRD-010`](../../../docs/requirements)
and issue [#197](https://github.com/laurigates/mcu-tinkering-lab/issues/197)
for the full feature description.

## Status

Firmware builds via the shared ESP-IDF container pipeline.  The audio path
(I2S RX, I2S TX, native touch, PSRAM clip storage) is **not yet verified on
hardware**; host tests exercise the dispatch logic only.

## Build & flash

```bash
just thinkpack-chatterbox::build
PORT=/dev/cu.usbmodemXXXX just thinkpack-chatterbox::flash-monitor
```

All ESP-IDF tooling runs inside `espressif/idf:v5.4`; no local install is
required.  See [`WIRING.md`](WIRING.md) for the pinout.

## Testing

Host-testable logic lives in `packages/components/thinkpack-behaviors`
(Echo Chamber dispatch) and `packages/components/thinkpack-audio` (state
machine, ring buffer, pitch-shift).  Run `make test` in either directory.
