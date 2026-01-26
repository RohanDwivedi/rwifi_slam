#!/bin/bash

set -e  # Exit on error

# ============================================================================
# Configuration
# ============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"
ISAAC_SIM_DIR="$SCRIPT_DIR/isaac-sim-6.0"
ISAAC_SIM_REPO="$ISAAC_SIM_DIR/IsaacSim"
ISAAC_SIM_VERSION="v6.0.0-dev"
BUILD_JOBS=12
REGISTRY_OWNER="${REGISTRY_OWNER:-rohandwivedi}"
DOCKER_IMAGE="ghcr.io/${REGISTRY_OWNER}/isaac-sim-6.0:latest"
ISAAC_SIM_REPO_URL="https://github.com/isaac-sim/IsaacSim.git"

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

print_info() {
    print_separator
    echo "$1"
    print_separator
    echo
}

# ============================================================================
# Argument Parsing
# ============================================================================

parse_arguments() {
    FORCE_BUILD=false

    for arg in "$@"; do
        case $arg in
            --force|-f)
                FORCE_BUILD=true
                ;;
            --help|-h)
                show_help
                exit 0
                ;;
            *)
                echo "Unknown option: $arg"
                echo "Use --help for usage information"
                exit 1
                ;;
        esac
    done
}

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo "Options:"
    echo "  --force, -f         Force rebuild even if _build folder exists"
    echo "  --help, -h          Show this help message"
}

# ============================================================================
# Dependency Installation
# ============================================================================

install_dependencies() {
    print_section "Installing build dependencies..."

    # Update package list
    sudo apt-get update

    # Install all required packages in one go
    sudo apt-get install -y \
        ccache \
        git-lfs \
        build-essential \
        make \
        cmake \
        gcc-11 \
        g++-11

    # Configure GCC/G++ 11 as alternatives
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100

    # Verify compiler versions
    echo "Verifying compiler versions:"
    gcc --version
    g++ --version
    echo
}

# ============================================================================
# Repository Management
# ============================================================================

prepare_repository() {
    print_section "Preparing Isaac Sim repository..."

    if [ "$FORCE_BUILD" = true ]; then
        echo "Force build enabled - fetching latest changes..."
    fi
    echo

    mkdir -p "$ISAAC_SIM_DIR"
    cd "$ISAAC_SIM_DIR"

    if [ ! -d "IsaacSim" ]; then
        echo "Cloning Isaac Sim repository..."
        git clone --recursive "$ISAAC_SIM_REPO_URL" IsaacSim
    else
        echo "Repository already exists, updating..."
        cd IsaacSim
        git fetch origin
        cd ..
    fi

    cd IsaacSim
    echo "Checking out Isaac Sim 6.0 patch version: $ISAAC_SIM_VERSION"
    git checkout --detach "$ISAAC_SIM_VERSION" || git checkout -f "$ISAAC_SIM_VERSION"
    git lfs install
    git lfs pull
    echo
}

# ============================================================================
# Build Functions
# ============================================================================

build_isaac_sim() {
    print_section "Building Isaac Sim..."
    # Note: --ccache uses settings from repo.toml [repo_build.distributed_build_services]
    # and doesn't take a directory argument
    ./build.sh --release -j "$BUILD_JOBS" --ccache -f -q
    print_info "Building Isaac Sim completed successfully"
}

build_docker_image() {
    print_section "Building docker image..."

    cd "$PROJECT_ROOT"

    CACHE_DIR="${CACHE_DIR:-$PROJECT_ROOT/.buildx-cache}"
    mkdir -p "${CACHE_DIR}"

    docker buildx build \
        --progress=plain \
        --tag "$DOCKER_IMAGE" \
        --file "$SCRIPT_DIR/dockerfile.isaac-sim-6.0" \
        --cache-from="type=local,src=${CACHE_DIR}" \
        --cache-to="type=local,dest=${CACHE_DIR},mode=max" \
        --load \
        --build-arg ISAAC_SIM_VERSION="$ISAAC_SIM_VERSION" \
        "$SCRIPT_DIR"

    print_info "Docker image build completed successfully!"
    echo "✔ Image $DOCKER_IMAGE Built"
    print_separator
    echo "To run the image, use the following command:"
    echo "docker run -it $DOCKER_IMAGE"
    print_separator
    echo "To push the image to the registry, use the following command:"
    echo "docker push $DOCKER_IMAGE"
    print_separator
    echo "To pull the image from the registry, use the following command:"
    echo "docker pull $DOCKER_IMAGE"
    print_separator
}

# ============================================================================
# Main Build Logic
# ============================================================================

build_isaac_sim_if_needed() {
    if [ ! -d "$ISAAC_SIM_REPO/_build" ] || [ "$FORCE_BUILD" = true ]; then
        if [ "$FORCE_BUILD" = true ]; then
            print_section "Building Isaac Sim 6.0 locally (FORCE MODE)..."
            echo "This will rebuild even if _build exists"
        else
            print_section "Building Isaac Sim 6.0 locally..."
        fi
        echo

        install_dependencies
        prepare_repository
        build_isaac_sim
        build_docker_image
    else
        print_info "_build folder already exists at ${ISAAC_SIM_REPO}/_build, skipping Isaac Sim build"
        echo "Use --force or -f to rebuild anyway"
        echo
        build_docker_image
    fi
}

# ============================================================================
# Main Entry Point
# ============================================================================

main() {
    parse_arguments "$@"
    build_isaac_sim_if_needed
}

main "$@"
