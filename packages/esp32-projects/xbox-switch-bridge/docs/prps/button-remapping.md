# PRP: Runtime Button Remapping

**Status**: Draft
**Feature**: FR11
**Date**: 2026-03-12

## Overview

Allow users to customize the Xbox-to-Switch button mapping at runtime instead of the current hardcoded mapping in `button_mapper.c`. The existing `credentials.h` pattern (gitignored header with compile-time values) could be extended, or an NVS-based runtime configuration could be added.

## Current State

Button mapping is hardcoded in `button_mapper_convert()`:
- Xbox A → Switch B (position-based swap)
- Xbox B → Switch A
- Xbox X → Switch Y
- Xbox Y → Switch X
- All other buttons: direct 1:1 mapping

Analog triggers use a fixed threshold of 128 for digital ZL/ZR activation.

## Implementation Options

### Option A: Compile-Time via credentials.h

Add `#define` overrides in `credentials.h`:
```c
#define BTN_MAP_XBOX_A  SW_BTN_A   // Override: Xbox A → Switch A (label-based)
#define BTN_MAP_TRIGGER_THRESHOLD 64
```

Pros: Simple, no runtime overhead, fits existing pattern.
Cons: Requires recompilation for changes.

### Option B: NVS Runtime Configuration

Store mapping table in NVS. Provide a CDC serial command interface to update mappings:
```
REMAP A=A B=B X=X Y=Y    # Label-based (no swap)
REMAP A=B B=A X=Y Y=X    # Position-based (default)
TRIGGER_THRESHOLD 64
```

Pros: No recompilation, survives power cycles.
Cons: More complex, CDC is not available in current architecture (no CDC-ACM on this project).

### Option C: WiFi Web Configuration

Host a simple web page on the SoftAP for remapping:
```
http://192.168.4.1/config
```

Pros: User-friendly, no special tools needed.
Cons: Adds HTTP server dependency, significant complexity.

## Recommendation

Start with Option A (compile-time) as it matches the existing `credentials.h` workflow and requires minimal code changes. Defer Options B/C until there's a clear user need for runtime remapping.

## Open Questions

- Is there demand for label-based mapping (Xbox A → Switch A) vs. position-based (current)?
- Should the analog trigger threshold be configurable per-trigger?
- Should stick dead zones be configurable?
