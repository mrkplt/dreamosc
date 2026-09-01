#!/usr/bin/env bash
# Fetch vendored third-party sources at their pinned commits, per
# vendor/manifest.txt. Invoked by `make vendor`.
#
#   ./vendor/fetch.sh          fetch/refresh vendored files
#   ./vendor/fetch.sh --check  verify on-disk files match upstream at the pinned
#                              SHA (non-zero exit if they drifted) -- this is what
#                              catches somebody editing a vendored file in place
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="$REPO_ROOT/vendor/manifest.txt"
CHECK_ONLY=0
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=1

status=0
while read -r url sha src dest; do
  # skip comments and blank lines
  [[ -z "${url:-}" || "$url" == \#* ]] && continue

  # https://github.com/OWNER/REPO -> raw.githubusercontent.com/OWNER/REPO
  raw="${url/github.com/raw.githubusercontent.com}/$sha/$src"
  target="$REPO_ROOT/$dest"
  tmp="$(mktemp)"

  if ! curl -fsSL "$raw" -o "$tmp"; then
    echo "!! FAILED to fetch $raw" >&2
    rm -f "$tmp"; status=1; continue
  fi

  if [[ $CHECK_ONLY -eq 1 ]]; then
    if [[ ! -f "$target" ]]; then
      echo "!! MISSING  $dest (run: make vendor)" >&2
      status=1
    elif ! cmp -s "$tmp" "$target"; then
      echo "!! DRIFTED  $dest differs from $url@${sha:0:7}:$src" >&2
      echo "            vendored files must not be edited in place -- either" >&2
      echo "            revert the edit or bump the SHA in vendor/manifest.txt" >&2
      status=1
    else
      echo "ok  $dest  ($url@${sha:0:7})"
    fi
    rm -f "$tmp"
  else
    mkdir -p "$(dirname "$target")"
    mv "$tmp" "$target"
    echo "fetched  $dest  <- $url@${sha:0:7}:$src"
  fi
done < "$MANIFEST"

exit $status
