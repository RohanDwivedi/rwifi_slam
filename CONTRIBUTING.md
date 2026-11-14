# Contributing

Thanks for your interest in rwifi_slam. Bug reports, fixes, and items from the
[roadmap](docs/ROADMAP.md) are all welcome.

## Before you start

- For anything larger than a small fix, open an issue first or comment on an existing one,
  so we can agree on the approach.
- Roadmap items are numbered (for example *M1.2*). Mention the number in your issue or PR.

## Development setup

1. Install the pre-commit hooks on the host:

   ```bash
   pip install pre-commit    # or: pipx install pre-commit
   pre-commit install
   ```

2. Start the development container, which builds the workspace on start:

   ```bash
   docker compose -f Docker/compose.yaml run --rm dev
   ```

3. Isaac Sim is only needed to run the simulator; see the
   [simulation README](src/simulation/README.md).

Without Docker, use a ROS 2 Jazzy installation on Ubuntu 24.04:

```bash
rosdep install --from-paths src --ignore-src -y
python3 -m pip install --break-system-packages -r src/rwifi_slam_ros2/requirements.txt
colcon build --symlink-install
```

## Code style

pre-commit enforces it, and CI runs the same hooks:

- Python is formatted with **Black** and **isort** (120-character lines), and linted with
  **flake8**.
- Docstrings follow the **Google** convention (checked by **pydocstyle**).
- YAML is checked by **yamllint**, GitHub workflows by **actionlint**.
- Files end with a newline, have no trailing whitespace, and use LF line endings.

The configuration is in [`Docker/pyproject.toml`](Docker/pyproject.toml),
[`Docker/.flake8`](Docker/.flake8), and [`Docker/.yamllint`](Docker/.yamllint). Run every hook
on every file with `pre-commit run --all-files`.

## Tests

Unit tests live in each package's `test/` directory and are plain pytest. They don't need a
ROS graph or the simulator.

```bash
colcon build
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Add or update tests with every behaviour change. A bug fix should come with a test that fails
without it.

## Pull requests

- Branch from `main`, and keep each PR to one topic.
- Fill in the PR template: what changed, why, and how you tested it.
- CI must pass: **Lint (pre-commit)** and **Build and test (ROS 2 Jazzy)**. PRs that touch
  `Docker/`, a `package.xml`, or `requirements.txt` also build both Docker images.
- If you change behaviour, parameters, topics, or commands, update the relevant README.

## Dependencies

- ROS and apt dependencies go in the package's `package.xml`, which is how `rosdep` and CI
  find them.
- Python packages with no rosdep key go in
  [`src/rwifi_slam_ros2/requirements.txt`](src/rwifi_slam_ros2/requirements.txt), pinned to a
  range you have tested. Don't add packages that apt already provides (such as numpy or scipy):
  pip can't replace Debian-managed packages.

## Releases (maintainers)

```bash
./scripts/bump-version.sh 0.14.0
git commit -am "Release 0.14.0"
git tag v0.14.0
git push origin main v0.14.0
```

The tag triggers the [release workflow](.github/workflows/release.yml). It publishes
`ghcr.io/rohandwivedi/rwifi_slam:0.14.0` (also tagged `0.14` and `latest`) and a GitHub
release with generated notes.

## License

By contributing, you agree that your contributions are licensed under the
[Apache License 2.0](LICENSE).
