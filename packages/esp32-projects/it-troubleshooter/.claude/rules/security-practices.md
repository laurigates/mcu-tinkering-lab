# Security Practices

**Confidence**: Very High — enforced by pre-commit hooks and gitleaks config
**Source**: security commit (2025-06-27), .gitleaks.toml, .pre-commit-config.yaml, CI permission hardening commits

## Credential Files

Never commit files containing credentials. Use `.example` templates instead:

```
credentials.h         ← gitignored (never commit)
credentials.h.example ← committed (template with placeholder values)
```

Pre-commit hooks automatically block: `credentials.h`, files ending in `.key`, `.secret`, `.token`.

All credential values go in `credentials.h` (gitignored). Non-sensitive defaults go in `sdkconfig.defaults`.

## Secret Scanning

`gitleaks` runs on every commit via pre-commit. The `.gitleaks.toml` allowlist is deliberately narrow — only two entries:
1. ESPHome `secrets.yaml` example (dummy values only)
2. Build artifact sodium source (test vectors)

Do not expand the allowlist without strong justification.

## GitHub Actions Permissions

Apply least privilege: set `permissions: {}` at the top-level workflow (deny all), then grant only what each job needs at the job level.

```yaml
# Top-level — deny by default
permissions: {}

jobs:
  build:
    permissions:
      contents: read        # only what this job needs
      pull-requests: write  # only if creating PR comments
```

Do not use top-level `permissions: write-all` or `permissions: contents: write` for workflow-level grants.

## API Token Storage

Store API tokens in `~/.api_tokens` (sourced by mise), never in the repository. Reference via environment variables in code and configuration.

```c
// In credentials.h (gitignored)
#define CLAUDE_API_KEY "sk-ant-..."
```
