#!/bin/bash
# Set the version of every ROS package in src/ (package.xml is the single source of truth;
# setup.py files read it).
#
# Usage: ./scripts/bump-version.sh <MAJOR.MINOR.PATCH>
#
# Then open a PR with the change. After it merges, tag the merge commit to publish:
#   git switch main && git pull
#   git tag -a v<version> -m "rwifi_slam <version>"
#   git push origin v<version>

set -euo pipefail

if [[ $# -ne 1 || ! "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Usage: $0 <MAJOR.MINOR.PATCH>" >&2
    exit 1
fi

NEW_VERSION="$1"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

CURRENT_VERSION="$("$PROJECT_ROOT/scripts/check-versions.sh" 2>/dev/null)" || {
    echo "Package versions are inconsistent; setting all of them to $NEW_VERSION." >&2
    CURRENT_VERSION=""
}

if [[ -n "$CURRENT_VERSION" ]]; then
    newest="$(printf '%s\n%s\n' "$CURRENT_VERSION" "$NEW_VERSION" | sort -V | tail -n 1)"
    if [[ "$NEW_VERSION" == "$CURRENT_VERSION" || "$newest" != "$NEW_VERSION" ]]; then
        echo "error: $NEW_VERSION is not newer than the current version $CURRENT_VERSION" >&2
        exit 1
    fi
fi

for manifest in "$PROJECT_ROOT"/src/*/package.xml; do
    sed -i -E "0,/<version>[^<]+<\/version>/s//<version>$NEW_VERSION<\/version>/" "$manifest"
done

"$PROJECT_ROOT/scripts/check-versions.sh" >/dev/null
echo "All packages: ${CURRENT_VERSION:-mixed} -> $NEW_VERSION"
