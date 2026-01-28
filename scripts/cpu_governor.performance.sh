#!/usr/bin/env bash
set -euo pipefail

UNIT_NAME=cpu-performance-governor.service
UNIT_PATH=/etc/systemd/system/$UNIT_NAME

CPUPWR="$(command -v cpupower || true)"
if [[ -z "${CPUPWR}" ]]; then
  echo "cpupower not found. Install it first, then re-run." >&2
  exit 1
fi

sudo tee "$UNIT_PATH" >/dev/null <<EOF
[Unit]
Description=Set CPU governor to performance (cpupower) for Isaac Sim
After=multi-user.target
ConditionPathExists=${CPUPWR}

[Service]
Type=oneshot
ExecStart=${CPUPWR} frequency-set -g performance
RemainAfterExit=yes

[Install]
WantedBy=multi-user.target
EOF

sudo systemctl daemon-reload
sudo systemctl enable --now "$UNIT_NAME"

echo "=========================================="
echo "CPU governor set to performance."
echo "You can check the current governor by running: cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor"
echo "=========================================="

read -p " Do you want to enable xhost for the current user (recommended for GUI applications on the container)? (y/n)" -n 1 -r
if [[ $REPLY =~ ^[Yy]$ ]]
then
    echo "=========================================="
    echo "Enabling xhost for the current user"
    echo "xhost +" >> ~/.bashrc
    echo "Xhost permission granted for the current user permanently."
    echo "=========================================="
fi