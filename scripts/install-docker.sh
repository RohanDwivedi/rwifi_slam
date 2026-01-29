#!/bin/bash
# Docker installation script for Ubuntu 24.04

set -e

echo "Installing Docker on Ubuntu 24.04..."

# Update package index
echo "Updating package index..."
sudo apt-get update

# Install prerequisites
echo "Installing prerequisites..."
sudo apt-get install -y \
    ca-certificates \
    curl \
    gnupg \
    lsb-release

# Add Docker's official GPG key
echo "Adding Docker's official GPG key..."
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add Docker repository
echo "Adding Docker repository..."
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Update package index again
echo "Updating package index with Docker repository..."
sudo apt-get update

# Install Docker Engine, CLI, and Containerd
echo "Installing Docker Engine..."
sudo apt-get install -y docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# Start and enable Docker service
echo "Starting Docker service..."
sudo systemctl start docker
sudo systemctl enable docker

# Add current user to docker group (so you can run docker without sudo)
echo "Adding user to docker group..."
sudo usermod -aG docker $USER

# Verify installation
echo ""
echo "Docker installation complete!"
echo ""
echo "Verifying installation..."
sudo docker --version
sudo docker run hello-world

echo ""
echo "=========================================="
echo "Docker has been successfully installed!"
echo "=========================================="
echo ""
echo "Note: You may need to log out and log back in (or run 'newgrp docker')"
echo "      for the group membership to take effect, so you can run docker"
echo "      commands without sudo."
echo ""
