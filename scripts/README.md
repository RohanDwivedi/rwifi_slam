# Scripts

This directory contains utility scripts for setting up and building the rwifi_slam development environment.

## Scripts Overview

### `build-docker.sh`
Builds Docker images for development or release.

**Usage:**
```bash
# Build development image (default)
./scripts/build-docker.sh dev

# Build release image
./scripts/build-docker.sh rel

# Default (builds dev if no argument provided)
./scripts/build-docker.sh
```

**Features:**
- Creates Docker BuildKit builder if needed
- Uses persistent local cache for faster rebuilds
- For `rel`: Automatically builds dev image first (since rel depends on dev)
- Interactive confirmation (skipped in CI/CD)
- Dev image: `rwifi_slam-dev`
- Rel image: `ghcr.io/rohandwivedi/rwifi_slam:latest`

**Environment Variables:**
- `BUILDER_NAME`: Docker BuildKit builder name (default: `rwifi_slam-builder`)
- `CACHE_DIR`: Cache directory (default: `.buildx-cache`)
- `GHCR_OWNER`: GitHub Container Registry owner (default: `rohandwivedi`)

**Examples:**
```bash
# Build dev image
GHCR_OWNER=myusername ./scripts/build-docker.sh dev

# Build rel image
GHCR_OWNER=myusername ./scripts/build-docker.sh rel
```

---

### `build-isaac-sim-local.sh`
Builds NVIDIA Isaac Sim 6.0 from source locally.

**Usage:**
```bash
./scripts/build-isaac-sim-local.sh [OPTIONS]
```

**Options:**
- `--force, -f`: Force rebuild even if `_build` folder exists
- `--clean-cache, -c`: Clean old Isaac Sim cache directories
- `--help, -h`: Show help message

**Features:**
- Clones Isaac Sim repository if needed
- Checks out Isaac Sim 6.0 patch version
- Installs build dependencies (GCC-11, CMake, etc.)
- Configures cache directories in `/workspace/.isaac-sim-cache`
- Builds with ccache for faster rebuilds
- Creates alias: `isaac-rwifi_slam`

**Example:**
```bash
# Normal build
./scripts/build-isaac-sim-local.sh

# Force rebuild
./scripts/build-isaac-sim-local.sh --force

# Clean old caches first
./scripts/build-isaac-sim-local.sh --clean-cache
```

**Requirements:**
- Sufficient disk space (50GB+ recommended)
- NVIDIA GPU with CUDA support
- Git LFS installed

**Cache Management:**
The script uses `/workspace/.isaac-sim-cache` for:
- Packman packages (`packman/`)
- Pip cache (`pip/`)
- UV cache (`uv/`)
- Omniverse data (`ov-data/`)

---

### `install-docker.sh`
Installs Docker Engine and Docker Compose on Linux systems.

**Usage:**
```bash
./scripts/install-docker.sh
```

**Features:**
- Installs Docker Engine from official repository
- Installs Docker Compose plugin
- Adds current user to `docker` group
- Configures Docker to start on boot

**Post-Installation:**
After installation, log out and log back in (or restart) for group changes to take effect.

**Verify Installation:**
```bash
docker --version
docker compose version
docker ps
```

---

### `install_nvidia_isaac_sim.sh`
Installs NVIDIA Driver 535 and NVIDIA Container Toolkit for Isaac Sim 6.0.

**Usage:**
```bash
./scripts/install_nvidia_isaac_sim.sh
```

**Features:**
- Installs NVIDIA Driver 535 (recommended for Isaac Sim 6.0)
- Installs NVIDIA Container Toolkit
- Configures Docker to use NVIDIA runtime
- Restarts Docker service

**Important Notes:**
- **REBOOT REQUIRED**: System must be rebooted after driver installation
- This script requires `sudo` privileges
- Verifies installation with `nvidia-smi` after reboot

**Post-Installation:**
```bash
# Reboot system
sudo reboot

# After reboot, verify:
nvidia-smi

# Verify Container Toolkit:
docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi
```

**Troubleshooting:**
- If driver installation fails, check NVIDIA compatibility
- Ensure secure boot is disabled (if applicable)
- Check kernel logs: `dmesg | grep -i nvidia`

---

## Common Workflows

### Initial Setup
```bash
# 1. Install Docker
./scripts/install-docker.sh
# Log out and back in

# 2. Install NVIDIA drivers
./scripts/install_nvidia_isaac_sim.sh
sudo reboot

# 3. Build Docker development image
./scripts/build-docker.sh dev

# 4. (Optional) Build Isaac Sim locally
./scripts/build-isaac-sim-local.sh
```

### Development Workflow
```bash
# Build/rebuild Docker development image
./scripts/build-docker.sh dev

# Run container
cd Docker
docker compose run dev
```

### Isaac Sim Development
```bash
# Build Isaac Sim (first time)
./scripts/build-isaac-sim-local.sh

# Rebuild Isaac Sim (if needed)
./scripts/build-isaac-sim-local.sh --force

# Clean old caches to free space
./scripts/build-isaac-sim-local.sh --clean-cache
```

## Script Permissions

All scripts should be executable:
```bash
chmod +x scripts/*.sh
```

## Error Handling

All scripts use `set -e` to exit on errors. If a script fails:
1. Check error messages in the output
2. Verify prerequisites are met
3. Check disk space and permissions
4. Review logs for detailed error information

## Contributing

When adding new scripts:
- Use `#!/bin/bash` shebang
- Include `set -e` for error handling
- Add help text with `--help` option
- Document environment variables
- Follow existing script patterns
