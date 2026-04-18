# Development Workflow

## TDD Cycle

Follow RED -> GREEN -> REFACTOR:
1. Write a failing test that defines desired behavior
2. Implement minimal code to make the test pass
3. Refactor while keeping tests green

## Commit Conventions

Use conventional commits: `feat:`, `fix:`, `docs:`, `chore:`, `refactor:`, `test:`, `ci:`, `build:`

Scope to component when applicable:
- `feat(game-logic): add multi-tag sequence support`
- `fix(rc522): correct SPI timing for tag reads`
- `docs(api): update Gemini TTS endpoint docs`

## ESP-IDF Specific

- Always use `sdkconfig.defaults` for configuration (not `sdkconfig`)
- Delete `sdkconfig` after modifying defaults before rebuilding
- Use `ESP_LOG*` macros for all logging (not printf)
- Check return values of ESP-IDF functions with `ESP_ERROR_CHECK` or explicit handling
