#!/usr/bin/env bash
# Verify that every file staged in <dir> is on GitHub release <tag> under the
# same name, and that each one downloads (HTTP 200).
#
#   verify-release-assets.sh <tag> <dir>
#
# Needs gh (GH_TOKEN), jq, curl, GNU find, and GITHUB_REPOSITORY (set in
# GitHub Actions). Used by both release jobs in .github/workflows/
# build-firmware.yml; see .claude/rules/release-publish-verification.md § 5.
set -euo pipefail

tag="${1:?usage: verify-release-assets.sh <tag> <dir>}"
dir="${2:?usage: verify-release-assets.sh <tag> <dir>}"
repo="${GITHUB_REPOSITORY:?GITHUB_REPOSITORY is not set}"

# Build the expected set from what we actually staged, by NAME —
# never by count. A count-only comparison is name-blind: it goes
# green if one expected file is silently swapped for an unrelated
# one (same total), and it goes red on an unrelated pre-existing
# release asset outside this branch's naming scheme (extra total).
mapfile -t expected_names < <(find "$dir" -mindepth 1 -type f -printf '%f\n' | sort)
if [ "${#expected_names[@]}" -eq 0 ]; then
  echo "::error::no files found under $dir/ — nothing was staged to verify"
  exit 1
fi

assets_json=$(gh api "repos/$repo/releases/tags/$tag" --jq '.assets') || {
  echo "::error::could not read the assets of release $tag"
  exit 1
}

missing=()
urls=()
for name in "${expected_names[@]}"; do
  url=$(printf '%s' "$assets_json" | jq -r --arg n "$name" '[.[] | select(.name == $n) | .browser_download_url] | first // empty')
  if [ -z "$url" ]; then
    missing+=("$name")
  else
    urls+=("$url")
  fi
done

if [ "${#missing[@]}" -gt 0 ]; then
  echo "::error::the following staged assets are not present on release $tag:"
  printf '  %s\n' "${missing[@]}"
  exit 1
fi

failed=()
for url in "${urls[@]}"; do
  code=$(curl -sSL --max-time 30 --retry 5 --retry-all-errors -o /dev/null -w '%{http_code}' "$url" || echo 000)
  echo "$code $url"
  if [ "$code" != "200" ]; then
    failed+=("$url -> $code")
  fi
done

if [ "${#failed[@]}" -gt 0 ]; then
  echo "The following release assets were not downloadable:"
  printf '  %s\n' "${failed[@]}"
  exit 1
fi
