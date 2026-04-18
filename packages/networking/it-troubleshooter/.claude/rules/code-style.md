# Code Style

**Confidence**: Very High — enforced by pre-commit hooks and clang-format config
**Source**: `.clang-format`, `.pre-commit-config.yaml`, multiple `fix(auto)` commits

## C/C++ Formatting

- **Style base**: Google with 4-space indent (overrides Google's 2-space default)
- **Line limit**: 100 characters (strict — critical for readability on OLED boards)
- **Brace style**: Linux (break before braces for control flow)
- **Pointer alignment**: Right (`int *ptr`, not `int* ptr`)
- **Enforced by**: `clang-format` v17+ via pre-commit; CI fails on violations

Do not manually format C/C++ — run `clang-format` or let pre-commit handle it.

## Python Formatting

- **Formatter**: `ruff-format` (not Black)
- **Linter**: `ruff` with auto-fix enabled
- **Type checker**: `mypy` (excludes test files)
- **Python version**: 3.11 (pinned in pre-commit)

## Function and Component Design

- Keep functions small and single-responsibility
- Prefer explicit initialization over constructor side effects:
  ```c
  // Good — explicit opt-in initialization
  wifi_manager_init();
  wifi_manager_connect();

  // Avoid — side effects in init() that always run
  ```
- This pattern prevents unwanted behavior in tests and alternative configurations

## Naming

- Public component functions: `{component_name}_{verb}_{noun}()` (e.g., `status_led_set_mode()`)
- Static (private) functions: no prefix required
- Log tags: `static const char *TAG = "component_name"` (matches component directory name)
- `#define` constants: `SCREAMING_SNAKE_CASE`

## Paths to Format Config

- `.clang-format` — C/C++ rules (repo root)
- `.pre-commit-config.yaml` — enforcement hooks (repo root)
