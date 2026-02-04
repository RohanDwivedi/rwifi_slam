#!/bin/bash

set -euo pipefail

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Use underscores to match Dockerfile 'FROM' and YAML requirements
DEV_IMAGE_NAME="rwifi_slam_dev"
GHCR_OWNER="${GHCR_OWNER:-rohandwivedi}"
REGISTRY_IMAGE="ghcr.io/${GHCR_OWNER}/rwifi_slam:latest"

BUILD_TYPE="${1:-rel}"

if [[ ! "$BUILD_TYPE" =~ ^(dev|rel)$ ]]; then
    echo "Error: Invalid build type '$BUILD_TYPE'. Use 'dev' or 'rel'."
    exit 1
fi

# ============================================================================
# Utility Functions
# ============================================================================

print_section() {
    echo "=========================================="
    echo "$1"
    echo "=========================================="
}

setup_buildx() {
    export DOCKER_BUILDKIT=1
    
    # In CI/GitHub Actions, use the default docker driver to ensure 
    # images are shared between build steps and accessible to 'docker run'
    if [[ -n "${GITHUB_ACTIONS:-}" ]]; then
        echo "CI detected: Using default docker driver..."
        docker buildx use default
    else
        # Local development: Ensure a custom builder exists for advanced features
        BUILDER_NAME="rwifi_slam-builder"
        if ! docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
            docker buildx create --name "${BUILDER_NAME}" --driver docker-container --use
        else
            docker buildx use "${BUILDER_NAME}"
        fi
    fi
}

# ============================================================================
# Build Logic
# ============================================================================

run_build() {
    local tag=$1
    local file=$2
    local msg=$3

    echo "--> $msg"
    docker buildx build \
        --progress=plain \
        --tag "$tag" \
        --file "$file" \
        --load \
        "$PROJECT_ROOT"
}

main() {
    print_section "Building rwifi_slam ($BUILD_TYPE)"

    # Skip prompt in CI
    if [[ -t 0 && -z "${GITHUB_ACTIONS:-}" ]]; then
        read -p "Continue? (y/N) " -n 1 -r; echo
        [[ ! $REPLY =~ ^[Yy]$ ]] && echo "Cancelled." && exit 1
    fi

    setup_buildx

    # Logic: If 'rel', we MUST build 'dev' first because Dockerfile.rel depends on it
    if [[ "$BUILD_TYPE" == "rel" ]]; then
        run_build "$DEV_IMAGE_NAME" "${PROJECT_ROOT}/Docker/dockerfile.dev" "Building Dev dependency..."
        run_build "$REGISTRY_IMAGE" "${PROJECT_ROOT}/Docker/dockerfile.rel" "Building Release image..."
    else
        run_build "$DEV_IMAGE_NAME" "${PROJECT_ROOT}/Docker/dockerfile.dev" "Building Dev image..."
    fi

    print_section "Build Completed Successfully"
    echo "Images available: $(docker images --format '{{.Repository}}:{{.Tag}}' | grep rwifi_slam | xargs)"
}

main "$@"
