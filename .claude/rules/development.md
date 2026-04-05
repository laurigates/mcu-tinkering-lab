# Development Workflow

## TDD Workflow

Follow RED -> GREEN -> REFACTOR:
1. Write a failing test that defines desired behavior
2. Implement minimal code to make the test pass
3. Refactor while keeping tests green

## Commit Conventions

Use conventional commits: `feat:`, `fix:`, `docs:`, `chore:`, `refactor:`, `test:`, `ci:`, `build:`

Include scope for ESP32 projects:
- `feat(robocar-main): add WiFi reconnection logic`
- `fix(robocar-camera): correct UART baud rate`

## Code Quality Gates

Before committing:
1. `just format` - Format all code
2. `just lint` - Lint all code
3. `pre-commit run --all-files` - Run all hooks

## Branch Strategy

- `main` - Production-ready code
- Feature branches from `main`
- PRs require CI passing
