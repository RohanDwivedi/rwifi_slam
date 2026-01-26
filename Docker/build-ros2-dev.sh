#!/bin/bash

set -euo pipefail

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
BUILDER_NAME="${BUILDER_NAME:-wildslam-builder}"
CACHE_DIR="${CACHE_DIR:-.buildx-cache}"
GHCR_OWNER="${GHCR_OWNER:-rohandwivedi}"
DOCKER_IMAGE="ghcr.io/${GHCR_OWNER}/wild_slam:latest"
DOCKERFILE="${SCRIPT_DIR}/dockerfile.ros2-dev"

# ============================================================================
# Utility Functions
# ============================================================================

print_separator() {
    echo "=========================================="
}

print_section() {
    print_separator
    echo "$1"
    print_separator
}

# ============================================================================
# Build Preparation
# ============================================================================

confirm_build() {
    # Skip prompt in non-interactive environments (CI/CD)
    if [[ -t 0 && -z "${CI:-}" && -z "${GITHUB_ACTIONS:-}" ]]; then
        read -p "Continue? (y/N) " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo "Build cancelled."
            exit 1
        fi
    else
        echo "Non-interactive mode detected - proceeding with build..."
    fi
}

print_diagnostics() {
    echo ""
    echo "Diagnostics (to verify WSL resources + BuildKit)..."
    echo "-------------------------------------------------"
    echo "WSL CPUs: $(nproc || true)"
    echo "WSL MemTotal: $(grep -i MemTotal /proc/meminfo 2>/dev/null || true)"
    echo "Docker version: $(docker version --format '{{.Server.Version}}' 2>/dev/null || docker version | head -n 20)"
    echo "Buildx version: $(docker buildx version 2>/dev/null || true)"
    echo ""
}

setup_buildx() {
    # Ensure we use BuildKit/buildx (stable)
    export DOCKER_BUILDKIT=1

    # Ensure a buildx builder exists and is selected (no-op if already present)
    if ! docker buildx inspect "${BUILDER_NAME}" >/dev/null 2>&1; then
        echo "Creating buildx builder: ${BUILDER_NAME}"
        docker buildx create --name "${BUILDER_NAME}" --use >/dev/null
    else
        docker buildx use "${BUILDER_NAME}" >/dev/null
    fi
}

prepare_cache() {
    # Persistent local cache directory (best on WSL ext4, not /mnt/c)
    mkdir -p "${CACHE_DIR}"
}

# ============================================================================
# Build Functions
# ============================================================================

build_docker_image() {
    echo ""
    echo "Starting build (with persistent cache)..."
    echo ""

    docker buildx build \
        --progress=plain \
        --tag "$DOCKER_IMAGE" \
        --file "$DOCKERFILE" \
        --cache-from="type=local,src=${CACHE_DIR}" \
        --cache-to="type=local,dest=${CACHE_DIR},mode=max" \
        --load \
        "$PROJECT_ROOT"
}

print_build_info() {
    echo ""
    print_section "Build completed successfully!"
    echo ""
    echo "✔ Image $DOCKER_IMAGE built"
    echo ""
    echo "Cache verification tips:"
    echo "  - Run this script twice."
    echo "  - On the second run, you should see many steps as 'CACHED'."
    echo "  - In --progress=plain output, BuildKit will report cache hits."
    echo ""
    echo "Run:"
    echo "  cd Docker"
    echo "  docker compose run dev"
    echo ""
    echo "Or push to GHCR:"
    echo "  docker push $DOCKER_IMAGE"
    echo ""
}

# ============================================================================
# Main Entry Point
# ============================================================================

main() {
    print_section "Building WiLD-SLAM Development Docker Image"

    confirm_build
    print_diagnostics
    setup_buildx
    prepare_cache
    build_docker_image
    print_build_info
}

main "$@"
