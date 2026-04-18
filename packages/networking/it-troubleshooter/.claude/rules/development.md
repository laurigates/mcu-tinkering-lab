# Development Workflow

## Build System

- All firmware compilation runs in Docker (`espressif/idf:v5.4`) via `just build`
- Flash and monitor run natively on the host (macOS USB passthrough is unreliable in containers)
- Use `just` recipes as the primary interface; run `just --list` to see all available recipes

### LSP Diagnostics Are False Positives on macOS

The host Clang/clangd LSP does not resolve ESP-IDF headers. All "file not found",
"unknown type name", and "undeclared identifier" diagnostics in `.c` and `.h` files
are false positives — ignore them entirely. Only `just build` (Docker, real compiler)
output matters for correctness.

## Commit Conventions

Use conventional commits: `feat:`, `fix:`, `docs:`, `chore:`, `refactor:`, `test:`, `ci:`, `build:`

Scope examples: `(usb_composite)`, `(status_led)`, `(main)`, `(build)`

## ESP-IDF Component Conventions

- Each component lives in `components/<name>/` with its own `CMakeLists.txt` and `idf_component.yml`
- Public headers go in `components/<name>/include/`
- Use `PRIV_REQUIRES` for internal dependencies, `REQUIRES` only for public API dependencies
- Prefix all public functions with the component name (e.g., `usb_composite_*`, `status_led_*`)
- Static functions for all internal helpers

## Credentials

- Copy `main/credentials.h.example` → `main/credentials.h` (gitignored)
- Never commit `credentials.h` or any file containing WiFi/API credentials
