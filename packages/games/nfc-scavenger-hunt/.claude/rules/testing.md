# Testing Requirements

## Embedded Testing Strategy

- Unit test pure logic functions (game_logic, prompt_builder) on host
- Use mocks for hardware-dependent code (RC522 driver, I2S audio, WiFi)
- Integration tests run on target hardware when available

## Coverage Expectations

- Game logic: high coverage (pure functions, easy to test)
- Driver code: test initialization and error paths
- Network code: mock HTTP responses and WiFi events

## Test Organization

- Tests in `test/` directory following ESP-IDF Unity test conventions
- Host-based tests use CMake CTest where applicable
