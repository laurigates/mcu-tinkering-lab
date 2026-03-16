#!/usr/bin/env bash
# Install project-specific tools for Claude Code remote (web) sessions.
# Runs as a SessionStart hook; exits immediately on local machines.
set -euo pipefail

if [ "${CLAUDE_CODE_REMOTE:-}" != "true" ]; then
  exit 0
fi

# ---------- pre-commit ----------
if ! command -v pre-commit >/dev/null 2>&1; then
  echo "Installing pre-commit..."
  pip install --quiet pre-commit
fi

# ---------- gitleaks v8.30.1 ----------
if ! command -v gitleaks >/dev/null 2>&1; then
  echo "Installing gitleaks v8.30.1..."
  tmp_dir=$(mktemp -d)
  curl -fsSL "https://github.com/gitleaks/gitleaks/releases/download/v8.30.1/gitleaks_8.30.1_linux_x64.tar.gz" \
    -o "$tmp_dir/gitleaks.tar.gz"
  tar -xzf "$tmp_dir/gitleaks.tar.gz" -C "$tmp_dir"
  mv "$tmp_dir/gitleaks" /usr/local/bin/gitleaks
  chmod +x /usr/local/bin/gitleaks
  rm -rf "$tmp_dir"
fi

# ---------- just (latest) ----------
if ! command -v just >/dev/null 2>&1; then
  echo "Installing just..."
  tmp_dir=$(mktemp -d)
  # Use the official install script which auto-detects platform
  curl -fsSL "https://just.systems/install.sh" | bash -s -- --to "$tmp_dir"
  mv "$tmp_dir/just" /usr/local/bin/just
  chmod +x /usr/local/bin/just
  rm -rf "$tmp_dir"
fi

echo "All tools installed successfully."
