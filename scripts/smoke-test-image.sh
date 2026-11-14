#!/bin/bash
# Smoke-test a release image: executables are installed, package versions are as expected,
# the nodes' Python dependencies import, and the SLAM node starts and keeps running.
#
# Usage: ./scripts/smoke-test-image.sh <image> [expected-version]

set -euo pipefail

if [[ $# -lt 1 || $# -gt 2 ]]; then
    echo "Usage: $0 <image> [expected-version]" >&2
    exit 1
fi

IMAGE="$1"
EXPECTED_VERSION="${2:-}"

docker run --rm -e EXPECTED_VERSION="$EXPECTED_VERSION" "$IMAGE" bash -c '
set -euo pipefail

check() { echo "--> $1"; }

check "executables"
ros2 pkg executables rwifi_slam_ros2 | grep -qw rwifi_slam_node
ros2 pkg executables wifi_rtt_interface | grep -qw wifi_rtt_publisher

check "package versions"
for package in rwifi_slam_ros2 wifi_rtt_interface wifi_rtt_msgs; do
    version=$(ros2 pkg xml "$package" --tag version)
    echo "    $package $version"
    if [ -n "$EXPECTED_VERSION" ] && [ "$version" != "$EXPECTED_VERSION" ]; then
        echo "error: $package is $version, expected $EXPECTED_VERSION" >&2
        exit 1
    fi
done

check "imports"
python3 -c "import rwifi_slam_ros2.rwifi_slam_ros2_node, wifi_rtt_interface.simulation_backend"

check "rwifi_slam_node starts and keeps running for 5 s"
status=0
timeout 5 ros2 run rwifi_slam_ros2 rwifi_slam_node > /tmp/node.log 2>&1 || status=$?
if [ "$status" -ne 124 ]; then
    cat /tmp/node.log >&2
    echo "error: rwifi_slam_node exited with status $status before the timeout" >&2
    exit 1
fi

echo "Smoke test passed."
'
