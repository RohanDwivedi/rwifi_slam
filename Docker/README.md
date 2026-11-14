# Docker

| File | Purpose |
|---|---|
| [`dockerfile.dev`](dockerfile.dev) | Development image `rwifi_slam:dev`. ROS 2 Jazzy desktop plus dependencies; source mounted at runtime |
| [`dockerfile.rel`](dockerfile.rel) | Release image `ghcr.io/rohandwivedi/rwifi_slam`. Workspace built into the image; Isaac Sim not included |
| [`compose.yaml`](compose.yaml) | `dev` and `release` services with GPU, X11, and host networking |
| [`entrypoint.sh`](entrypoint.sh) | Sources ROS 2, builds the workspace on first start if needed, then runs the command |
| `pyproject.toml`, `.flake8`, `.yamllint` | Lint configuration, used by pre-commit and copied into the dev image for editors |

Both images use the repository root as the build context, so run commands from there.
[`../.dockerignore`](../.dockerignore) keeps the local Isaac Sim build and caches out of the
context.

## Development image

```bash
docker compose -f Docker/compose.yaml run --rm dev
```

Compose builds `rwifi_slam:dev` if it is missing; to rebuild it, run
`./scripts/build-docker.sh dev`. The container:

- mounts `src/` at `/ros2_ws/src`, including the local Isaac Sim build in
  `src/simulation/isaac_sim_dev/isaac-sim-6.0`;
- builds the workspace with `colcon build --symlink-install` on start, so Python changes take
  effect without rebuilding;
- runs as root with host networking, the GPU, and the host's X11 socket.

Run `xhost +local:` on the host once per session so GUI applications (RViz, Isaac Sim) can
open windows.

VS Code users can open the repository and run **Dev Containers: Reopen in Container**, which
uses [`../.devcontainer/devcontainer.json`](../.devcontainer/devcontainer.json).

## Release image

Published by [`release.yml`](../.github/workflows/release.yml) for every version tag:

```bash
docker pull ghcr.io/rohandwivedi/rwifi_slam:latest
docker compose -f Docker/compose.yaml run --rm release
```

Tags are `<major>.<minor>.<patch>`, `<major>.<minor>`, and `latest`. `latest` follows the newest
release, so re-publishing an older tag doesn't move it. To build the image locally, run
`./scripts/build-docker.sh rel`, then check it with `./scripts/smoke-test-image.sh`. Isaac Sim isn't included; to simulate, mount a local
build at `/ros2_ws/src/simulation/isaac_sim_dev/isaac-sim-6.0`.

## Environment

| Variable | Value | Notes |
|---|---|---|
| `RMW_IMPLEMENTATION` | `rmw_fastrtps_cpp` | Set in `compose.yaml`; `rmw_zenoh_cpp` is also installed |
| `ROS_DOMAIN_ID` | unset (0) | Export it in the container, or add it to `compose.yaml`, to isolate the robot's traffic |
| `NVIDIA_VISIBLE_DEVICES`, `NVIDIA_DRIVER_CAPABILITIES` | `all` | GPU access through the NVIDIA Container Toolkit |
| `DISPLAY` | from host | X11 forwarding |

## Troubleshooting

- **GPU missing in the container:** check `docker run --rm --gpus all ubuntu nvidia-smi`. If
  that fails, install the toolkit with `scripts/install-nvidia-isaac-sim-dependencies.sh`.
- **The workspace build fails on start:** the entrypoint prints a warning and still opens a
  shell. Fix the error, then run `colcon build --symlink-install`.
- **No windows appear:** run `xhost +local:` on the host, and check `echo $DISPLAY` inside the
  container.
