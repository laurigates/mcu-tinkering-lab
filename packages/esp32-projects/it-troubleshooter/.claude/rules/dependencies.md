# Dependencies

**Confidence**: High — derived from build system and CI patterns
**Source**: docker-compose.yml, idf_component.yml files, CI workflow analysis

## ESP-IDF Version

Pin to a specific ESP-IDF version in `sdkconfig.defaults` and `docker-compose.yml`. The current project uses ESP-IDF v5.4.

```
docker image: espressif/idf:v5.4
CONFIG_IDF_TARGET=esp32s3
```

Do not use `latest` — ESP-IDF has breaking API changes between minor versions.

## IDF Component Manager

Declare dependencies in `idf_component.yml` using exact or semver-compatible ranges:

```yaml
# Good — semver constraint
dependencies:
  espressif/esp_tinyusb: "^1.0.0"
  espressif/led_strip: ">=2.0.0"
```

- `managed_components/` is gitignored — downloaded by IDF at build time
- Do not vendor managed components into the repository
- `EXTRA_COMPONENT_DIRS "components"` in root `CMakeLists.txt` for local components

## REQUIRES vs PRIV_REQUIRES

In component `CMakeLists.txt`:
- `REQUIRES` — only for dependencies used in public headers (`include/`)
- `PRIV_REQUIRES` — for dependencies used only in `.c` implementation files

```cmake
idf_component_register(
    SRCS "status_led.c"
    INCLUDE_DIRS "include"
    REQUIRES led_strip       # used in public header? No → should be PRIV_REQUIRES
    PRIV_REQUIRES esp_timer  # only used in .c file
)
```

Missing `esp_timer` in `PRIV_REQUIRES` is a recurring issue — always declare it explicitly (see commit 861a908).

## Python Dependencies

- **Package manager**: `uv` for Python projects (not pip directly)
- **Lock file**: Committed; use `uv sync` to install
- **Tool management**: `uv run` for one-off tools, `uv tool install` for persistent tools

## GitHub Actions Dependencies

Updated via Dependabot automatically. Current pinned versions:
- `actions/checkout@v6`
- `actions/cache@v5`
- `actions/setup-python@v6`
- `actions/upload-artifact@v7`
- `codecov/codecov-action@v5`

Use SHA pinning for external actions in security-sensitive workflows.
