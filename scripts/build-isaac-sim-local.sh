#!/bin/bash
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ISAAC_SIM_DIR="$SCRIPT_DIR/../src/simulation/isaac_sim_dev/isaac-sim-6.0"
ISAAC_SIM_REPO="$ISAAC_SIM_DIR/IsaacSim"
ISAAC_SIM_VERSION="v6.0.0-dev"
ISAAC_SIM_REPO_URL="https://github.com/isaac-sim/IsaacSim.git"
BUILD_JOBS=12

QUIET=false
VERBOSE=false
REBUILD_MODE="none"   # none | rebuild | fast-rebuild

parse_arguments() {
    while [ $# -gt 0 ]; do
        case "$1" in
            -x|--rebuild) REBUILD_MODE="rebuild"; shift ;;
            -f|--fast-rebuild) REBUILD_MODE="fast-rebuild"; shift ;;
            -q|--quiet) QUIET=true; shift ;;
            -v|--verbose) VERBOSE=true; shift ;;
            -h|--help) show_help; exit 0 ;;
            *)
                echo "Unknown option: $1"
                echo "Use --help for usage"
                exit 1
                ;;
        esac
    done

    if [ "$QUIET" = true ] && [ "$VERBOSE" = true ]; then
        echo "Error: --quiet and --verbose are mutually exclusive"
        exit 1
    fi
}

show_help() {
    echo "Usage: $0 [OPTIONS]"
    echo "  -x, --rebuild         Clean repo before building (build.sh --rebuild)"
    echo "  -f, --fast-rebuild    Partially clean before building (build.sh --fast-rebuild)"
    echo "  -q, --quiet           Suppress build output (build.sh --quiet)"
    echo "  -v, --verbose         Verbose build output (build.sh --verbose)"
    echo "  -h, --help            Show this help"
}

install_dependencies() {
    echo "=========================================="
    echo "Installing build dependencies..."
    echo "=========================================="
    sudo apt-get update
    sudo apt-get install -y ccache git-lfs build-essential make cmake gcc-11 g++-11
    sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-11 100
    sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-11 100
    gcc --version && g++ --version
    echo
}

prepare_repository() {
    echo "=========================================="
    echo "Preparing Isaac Sim repository..."
    echo "=========================================="
    mkdir -p "$ISAAC_SIM_DIR"
    cd "$ISAAC_SIM_DIR"

    if [ ! -d "IsaacSim" ]; then
        git clone --recursive "$ISAAC_SIM_REPO_URL" IsaacSim
    else
        cd IsaacSim
        git fetch origin
        cd ..
    fi

    cd IsaacSim
    git checkout --detach "$ISAAC_SIM_VERSION" 2>/dev/null || git checkout -f "$ISAAC_SIM_VERSION"
    git lfs install
    git lfs pull
    echo
}

execute_build_isaac_sim() {
    echo "=========================================="
    echo "Building Isaac Sim..."
    echo "=========================================="
    cd "$ISAAC_SIM_REPO"

    args=(--release --jobs "$BUILD_JOBS" --ccache)

    case "$REBUILD_MODE" in
        rebuild) args+=(--rebuild) ;;
        fast-rebuild) args+=(--fast-rebuild) ;;
        none) ;;
        *) echo "Invalid REBUILD_MODE: $REBUILD_MODE"; exit 1 ;;
    esac

    if [ "$QUIET" = true ]; then
        args+=(--quiet)
    elif [ "$VERBOSE" = true ]; then
        args+=(--verbose)
    fi

    yes yes | ./build.sh "${args[@]}"
    
    echo "Build completed successfully."
    echo
}


build_isaac_sim() {
    if [ -d "$ISAAC_SIM_REPO/_build" ] && [ "$REBUILD_MODE" = "none" ]; then
        echo "=========================================="
        echo "_build exists at $ISAAC_SIM_REPO/_build — skipping build"
        echo "Use -x/--rebuild or -f/--fast-rebuild to build anyway"
        echo "=========================================="
        echo
        return 0
    fi

    echo "=========================================="
    echo "Building Isaac Sim 6.0 locally"
    echo "=========================================="
    echo

    install_dependencies
    prepare_repository
    execute_build_isaac_sim

    echo "Isaac Sim build completed successfully."
    echo "=========================================="
}

build_isaac_sim
