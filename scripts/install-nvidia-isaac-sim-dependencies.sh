#!/bin/bash
set -euo pipefail

DRIVER_BRANCH="580"

echo "=========================================="
echo "Installing NVIDIA Driver ${DRIVER_BRANCH} + NVIDIA Container Toolkit"
echo "=========================================="
echo

sudo apt-get update
sudo apt-get install -y --no-install-recommends curl gnupg2 ca-certificates

echo
echo "Removing older NVIDIA packages (best-effort)..."
sudo apt-get remove -y \
  "nvidia-driver-*" \
  "nvidia-utils-*" \
  "libnvidia-*" \
  "xserver-xorg-video-nvidia-*" \
  "nvidia-dkms-*" \
  "nvidia-kernel-common-*" \
  "nvidia-kernel-source-*" \
  "nvidia-compute-utils-*" \
  "nvidia-fabricmanager-*" \
  "nvidia-settings" \
  "nvidia-prime" \
  >/dev/null 2>&1 || true
sudo apt-get autoremove -y

echo
echo "Installing NVIDIA driver ${DRIVER_BRANCH}..."
sudo apt-get install -y "nvidia-driver-${DRIVER_BRANCH}" "nvidia-utils-${DRIVER_BRANCH}"

echo
echo "Installing NVIDIA Container Toolkit (APT repo)..."

sudo install -d -m 0755 /usr/share/keyrings

sudo rm -f /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -fsSL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list >/dev/null

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

echo
echo "Configuring Docker NVIDIA runtime..."
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker || true

echo
echo "=========================================="
echo "Installation complete!"
echo "=========================================="
echo
echo "Reboot required:"
echo "  sudo reboot"
echo
echo "After reboot verify:"
echo "  nvidia-smi"
echo
echo "Verify NVIDIA Container Toolkit:"
echo "  docker run --rm --gpus all nvidia/cuda:11.0.3-base-ubuntu20.04 nvidia-smi"
echo
