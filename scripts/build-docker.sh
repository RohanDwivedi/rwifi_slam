#!/bin/bash
# Build the development or release Docker image.
#
# Usage: ./scripts/build-docker.sh [dev|rel]   (default: dev)
#
#   dev -> rwifi_slam:dev                         (source mounted at runtime)
#   rel -> ghcr.io/<GHCR_OWNER>/rwifi_slam:latest (workspace built into the image)

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

GHCR_OWNER="${GHCR_OWNER:-rohandwivedi}"
DEV_IMAGE="rwifi_slam:dev"
REL_IMAGE="ghcr.io/${GHCR_OWNER}/rwifi_slam:latest"

BUILD_TYPE="${1:-dev}"

case "$BUILD_TYPE" in
    dev) IMAGE="$DEV_IMAGE" ;;
    rel) IMAGE="$REL_IMAGE" ;;
    *)
        echo "Error: invalid build type '$BUILD_TYPE'. Use 'dev' or 'rel'." >&2
        exit 1
        ;;
esac

echo "Building $IMAGE from Docker/dockerfile.$BUILD_TYPE"

# Ask before a long build when run interactively.
if [[ -t 0 && -z "${CI:-}" ]]; then
    read -p "Continue? (y/N) " -n 1 -r
    echo
    [[ $REPLY =~ ^[Yy]$ ]] || { echo "Cancelled."; exit 1; }
fi

DOCKER_BUILDKIT=1 docker build \
    --progress=plain \
    --file "$PROJECT_ROOT/Docker/dockerfile.$BUILD_TYPE" \
    --tag "$IMAGE" \
    "$PROJECT_ROOT"

echo "Built $IMAGE"
