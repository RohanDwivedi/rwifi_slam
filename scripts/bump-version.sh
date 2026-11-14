#!/bin/bash
# Set the rwifi_slam_ros2 version in package.xml and setup.py.
#
# Usage: ./scripts/bump-version.sh <major.minor.patch>
#
# Then commit, tag, and push to trigger .github/workflows/release.yml:
#   git commit -am "Release <version>"
#   git tag v<version>
#   git push origin main v<version>

set -euo pipefail

if [[ $# -ne 1 || ! "$1" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Usage: $0 <major.minor.patch>" >&2
    exit 1
fi

NEW_VERSION="$1"
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE_XML="$PROJECT_ROOT/src/rwifi_slam_ros2/package.xml"
SETUP_PY="$PROJECT_ROOT/src/rwifi_slam_ros2/setup.py"

CURRENT_VERSION=$(grep -oP '<version>\K[^<]+' "$PACKAGE_XML")

sed -i "s|<version>$CURRENT_VERSION</version>|<version>$NEW_VERSION</version>|" "$PACKAGE_XML"
sed -i "s|version=\"$CURRENT_VERSION\"|version=\"$NEW_VERSION\"|" "$SETUP_PY"

grep -q "<version>$NEW_VERSION</version>" "$PACKAGE_XML"
grep -q "version=\"$NEW_VERSION\"" "$SETUP_PY"

echo "rwifi_slam_ros2: $CURRENT_VERSION -> $NEW_VERSION"
