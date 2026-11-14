#!/bin/bash
# Check that every ROS package in src/ has the same version, and optionally that it matches a tag.
#
# Usage: ./scripts/check-versions.sh [vX.Y.Z]
#
# Prints the shared version on success. Run by CI on every change and by the release workflow
# against the pushed tag.

set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TAG="${1:-}"

error() {
    if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
        echo "::error::$1"
    else
        echo "error: $1" >&2
    fi
}

versions=()
for manifest in "$PROJECT_ROOT"/src/*/package.xml; do
    package="$(basename "$(dirname "$manifest")")"
    version="$(grep -oP '<version>\K[^<]+' "$manifest" | head -n 1)"
    echo "$package: $version" >&2
    versions+=("$version")
done

status=0
expected="${versions[0]}"

for version in "${versions[@]}"; do
    if [[ "$version" != "$expected" ]]; then
        error "packages have different versions; run ./scripts/bump-version.sh to set them all"
        status=1
        break
    fi
done

# REP 149: package versions are exactly three dot-separated integers.
if [[ ! "$expected" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    error "version '$expected' is not MAJOR.MINOR.PATCH"
    status=1
fi

if [[ -n "$TAG" && "$TAG" != "v$expected" ]]; then
    error "tag '$TAG' does not match package version '$expected' (expected tag 'v$expected')"
    status=1
fi

[[ $status -eq 0 ]] && echo "$expected"
exit $status
