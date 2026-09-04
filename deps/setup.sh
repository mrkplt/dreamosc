#!/usr/bin/env bash
# Clone/checkout/verify the pinned build dependencies (libDaisy, DaisySP) per
# deps/manifest.txt. These are whole repos we build against, cloned beside the
# project and gitignored -- the manifest is the only record of which commit.
#
#   ./deps/setup.sh          clone if missing, then checkout each repo to its
#                            pinned SHA (fetches the SHA first if needed)
#   ./deps/setup.sh --check  verify each working clone is AT its pinned SHA and
#                            clean; non-zero exit if it drifted (a build gate,
#                            mirroring vendor-check for files)
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
MANIFEST="$REPO_ROOT/deps/manifest.txt"
CHECK_ONLY=0
[[ "${1:-}" == "--check" ]] && CHECK_ONLY=1

status=0
while read -r name url sha dir _rest; do
  [[ -z "${name:-}" || "$name" == \#* ]] && continue
  path="$REPO_ROOT/$dir"

  if [[ $CHECK_ONLY -eq 1 ]]; then
    if [[ ! -d "$path/.git" ]]; then
      echo "!! MISSING  $dir (run: make deps)" >&2
      status=1; continue
    fi
    have="$(git -C "$path" rev-parse HEAD)"
    if [[ "$have" != "$sha" ]]; then
      echo "!! DRIFTED  $dir at ${have:0:7}, pinned ${sha:0:7} -- run: make deps" >&2
      echo "            (or update the SHA in deps/manifest.txt to take a new upstream)" >&2
      status=1
    else
      echo "ok  $dir  (${sha:0:7})"
    fi
  else
    if [[ ! -d "$path/.git" ]]; then
      echo "cloning  $dir <- $url"
      git clone --recursive "$url" "$path"
    fi
    # Fetch the exact SHA if we don't have it, then check it out.
    if ! git -C "$path" cat-file -e "$sha^{commit}" 2>/dev/null; then
      git -C "$path" fetch --tags origin
    fi
    git -C "$path" checkout -q "$sha"
    git -C "$path" submodule update --init --recursive
    echo "pinned  $dir -> ${sha:0:7}"
  fi
done < "$MANIFEST"

exit $status
