# Scripts

| Script | Run on | Purpose |
|---|---|---|
| [`build-docker.sh`](build-docker.sh) | host | Build the `dev` (default) or `rel` image |
| [`build-isaac-sim-local.sh`](build-isaac-sim-local.sh) | dev container | Clone and build Isaac Sim 6.0 into `src/simulation/isaac_sim_dev/isaac-sim-6.0` |
| [`bump-version.sh`](bump-version.sh) | host | Set the `rwifi_slam_ros2` version before tagging a release |
| [`install-docker.sh`](install-docker.sh) | host | Install Docker Engine and the Compose plugin on Ubuntu 24.04 |
| [`install-nvidia-isaac-sim-dependencies.sh`](install-nvidia-isaac-sim-dependencies.sh) | host | Install the NVIDIA 580 driver and the NVIDIA Container Toolkit |
| [`cpu_governor.performance.sh`](cpu_governor.performance.sh) | host | Pin the CPU governor to `performance` for Isaac Sim |

## `build-docker.sh`

```bash
./scripts/build-docker.sh          # rwifi_slam:dev
./scripts/build-docker.sh rel      # ghcr.io/$GHCR_OWNER/rwifi_slam:latest
```

`GHCR_OWNER` defaults to `rohandwivedi`. The script asks for confirmation when run
interactively.

## `build-isaac-sim-local.sh`

Run this inside the dev container. It installs build tools with `apt`, sets gcc/g++ 11 as the
default compilers, clones Isaac Sim `v6.0.0-dev` with Git LFS, and builds it.

| Option | Effect |
|---|---|
| `-f`, `--fast-rebuild` | Partially clean, then build |
| `-x`, `--rebuild` | Clean, then build |
| `-q`, `--quiet` / `-v`, `--verbose` | Build output level |

## `bump-version.sh`

```bash
./scripts/bump-version.sh 0.14.0
git commit -am "Release 0.14.0"
git tag v0.14.0
git push origin main v0.14.0
```

Pushing the tag runs the [release workflow](../.github/workflows/release.yml). It checks that
the tag matches `package.xml`, publishes the image to GHCR, and creates a GitHub release.

## Host setup scripts

These change system configuration and need `sudo`. Read them before running.

- **`install-docker.sh`** adds Docker's apt repository, installs Docker, and adds you to the
  `docker` group. Log out and back in afterwards.
- **`install-nvidia-isaac-sim-dependencies.sh`** *removes any installed NVIDIA driver*,
  installs driver 580 and the container toolkit, and configures Docker's NVIDIA runtime.
  Reboot afterwards.
- **`cpu_governor.performance.sh`** installs a systemd unit that sets the `performance`
  governor at boot (needs `cpupower`). It can also add `xhost +local:` to `~/.bashrc`, which
  lets local containers open windows.
