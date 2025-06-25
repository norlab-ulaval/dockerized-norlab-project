# Installation Guide

Complete installation instructions for Dockerized-NorLab Project application (DNA) on all supported platforms.

## Table of Contents

- [Overview](#overview)
- [System Requirements](#system-requirements)
- [Quick Installation](#quick-installation)
- [Installation Methods](#installation-methods)
- [Platform-Specific Instructions](#platform-specific-instructions)
- [Post-Installation Setup](#post-installation-setup)
- [Verification](#verification)
- [Troubleshooting](#troubleshooting)
- [Uninstallation](#uninstallation)

## Overview

DNA can be installed using different methods depending on your system configuration and preferences. The installation process sets up the DNA command-line tool and configures your system to use containerized robotic development environments.

## System Requirements

### Core Requirements

- **Git**: Version control system
- **Docker Engine**: Version 20.10 or later with BuildKit support
- **Docker Compose**: Plugin version (v2) or standalone
- **Internet connection**: Required for initial setup and image downloads

### Platform Support

| Platform       | Architecture | Status |
|----------------|------------|---------|
| NVIDIA Jetson (L4T) | ARM64 | ✅ Fully supported |
| macOS          | ARM64 (Apple Silicon) | ✅ Fully supported |
| Linux          | x86_64 | ✅ Fully supported |
| Linux          | ARM64 | ⚠️ Not supported (for now, will be implemented if requested) |

### Optional Requirements

- **NVIDIA Docker**: For GPU acceleration support
- **Docker Buildx**: For multi-architecture builds
- **SSH**: For remote development workflows

## Quick Installation

For most users, the standard installation is recommended:

```bash
# Clone the repository
git clone --recurse-submodules https://github.com/norlab-ulaval/dockerized-norlab-project.git
cd dockerized-norlab-project

# Install DNA (creates system-wide symlink)
bash install.bash

# Verify installation
dna --help
```

## Installation Methods

### Method 1: System-Wide Installation (Recommended)

Creates a symlink in `/usr/local/bin/dna` for system-wide access:

```bash
bash install.bash
```

**Advantages:**
- Available from any directory
- Works for all users
- Standard Unix convention

**Requirements:**
- `/usr/local/bin` directory must exist
- Sudo privileges for symlink creation

### Method 2: User-Specific Installation

Adds DNA to your personal PATH via `~/.bashrc`:

```bash
bash install.bash --add-dna-path-to-bashrc
```

**Advantages:**
- No sudo privileges required
- User-specific configuration
- Works when `/usr/local/bin` is not available

**Requirements:**
- `~/.bashrc` file must exist
- Bash shell environment
- (zsh shell) `~/.bashrc` must be sourced by your `~/.zshrc`

### Method 3: Manual Path Management

Skip automatic installation and manage paths manually:

```bash
bash install.bash --skip-system-wide-symlink-install
```

**Use cases:**
- Custom deployment scripts
- Container environments
- Advanced users with specific requirements

### Method 4: Non-Interactive Installation

Bypass all prompts for automated deployments:

```bash
bash install.bash --yes
```

**Use cases:**
- CI/CD pipelines
- Automated provisioning
- Docker container builds

## Platform-Specific Instructions

### Linux (Ubuntu/Debian)

> **Note**: Docker Engine installation is handled automatically by `install.bash`. Manual installation is only needed for troubleshooting.

1. **(Optional) Install NVIDIA Docker (for GPU support):**
   ```bash
   distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
   curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
   curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list

   sudo apt-get update && sudo apt-get install -y nvidia-docker2
   sudo systemctl restart docker
   ```

2. **Install DNA:**
   ```bash
   git clone --recurse-submodules https://github.com/norlab-ulaval/dockerized-norlab-project.git
   cd dockerized-norlab-project
   bash install.bash
   ```
3. Follow post-installation setup instructions

### macOS

1. **Install Docker Desktop:**
   - Download from [Docker Desktop for Mac](https://docs.docker.com/desktop/mac/install/)
   - Install and start Docker Desktop
   - Ensure Docker Compose is enabled in settings

2. **Install DNA:**
   ```bash
   git clone --recurse-submodules https://github.com/norlab-ulaval/dockerized-norlab-project.git
   cd dockerized-norlab-project
   bash install.bash
   ```
3. Follow post-installation setup instructions

### NVIDIA Jetson (L4T/ARM64)
> **Note**: Docker Engine and nvidia-docker come pre-installed on Jetson

1. **Install DNA:**
   ```bash
   git clone --recurse-submodules https://github.com/norlab-ulaval/dockerized-norlab-project.git
   cd dockerized-norlab-project
   bash install.bash
   ```
2. Follow post-installation setup instructions

## Post-Installation Setup


### User Permissions (L4T and Linux)

Apply Docker group change without login out: execute 
```bash
newgrp docker
```
Restart the docker daemon to apply changes: execute 
```bash
sudo systemctl restart docker
```

### Environment Variables (Optional)

If using the bashrc installation method, reload your shell:

```bash
source ~/.bashrc
```

### Docker Buildx Setup (Optional)

Create a multi-architecture docker builder. Execute the following comands:
```bash
docker buildx create --name local-builder-multiarch-virtual --driver docker-container --platform linux/amd64,linux/arm64 --bootstrap --use
docker buildx ls
```

## Verification

### Basic Verification

```bash
# Check DNA is available
dna --help

# Check Docker is working
docker --version
docker compose version

# Test DNA with a sample project
git clone https://github.com/norlab-ulaval/dockerized-norlab-project-mock-EMPTY.git
cd dockerized-norlab-project-mock-EMPTY
dna init
```

### GPU Support Verification (if applicable)

```bash
# Test NVIDIA Docker
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

## Troubleshooting

### Common Issues

#### "dna: command not found"

**Problem**: DNA command is not in PATH.

**Solutions:**
1. **System-wide installation**: Check if symlink exists:
   ```bash
   ls -la /usr/local/bin/dna
   ```
   If missing, reinstall:
   ```bash
   bash install.bash
   ```

2. **Bashrc installation**: Reload shell configuration:
   ```bash
   source ~/.bashrc
   ```

#### Permission Denied

**Problem**: Cannot create symlink in `/usr/local/bin`.

**Solutions:**
1. **Create directory** (if missing):
   ```bash
   sudo mkdir -p /usr/local/bin
   ```

2. **Use alternative installation**:
   ```bash
   bash install.bash --add-dna-path-to-bashrc
   ```

#### Docker Permission Denied

**Problem**: Cannot run Docker commands without sudo.

**Solution**: Add user to docker group:
```bash
sudo usermod -aG docker $USER
newgrp docker
sudo systemctl restart docker
```

#### Submodule Issues

**Problem**: Missing submodules or outdated dependencies.

**Solution**: Update submodules:
```bash
git submodule update --init --recursive
```

### Platform-Specific Issues

#### macOS: Docker Desktop Not Running

**Problem**: Docker commands fail.

**Solution**: Start Docker Desktop application.

#### Linux: NVIDIA Docker Issues

**Problem**: GPU not accessible in containers.

**Solution**: Restart Docker daemon:
```bash
sudo systemctl restart docker
```

#### Jetson: Architecture Mismatch

**Problem**: Wrong architecture images.

**Solution**: Ensure you're using ARM64-compatible base images.

#### Linux: Manual Docker Engine Installation

**Problem**: Docker Engine installation issues or need for manual installation.

**Solution**: Install Docker Engine manually (this step is normally executed automatically by `install.bash`):

```bash
# Add Docker's official GPG key
sudo apt-get update
sudo apt-get install ca-certificates curl gnupg
sudo install -m 0755 -d /etc/apt/keyrings
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /etc/apt/keyrings/docker.gpg
sudo chmod a+r /etc/apt/keyrings/docker.gpg

# Add repository
echo \
  "deb [arch="$(dpkg --print-architecture)" signed-by=/etc/apt/keyrings/docker.gpg] https://download.docker.com/linux/ubuntu \
  "$(. /etc/os-release && echo "$VERSION_CODENAME")" stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker
sudo apt-get update
sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin
```

## Uninstallation

### Remove System-Wide Installation

```bash
# Remove symlink
sudo rm -f /usr/local/bin/dna

# Remove repository (optional)
rm -rf /path/to/dockerized-norlab-project
```

### Remove Bashrc Installation

```bash
# Edit ~/.bashrc and remove DNA-related lines:
# # >>>> Dockerized-NorLab Project (start)
# export _DNA_PATH="/path/to/dna/bin"
# export PATH="$PATH:$_DNA_PATH"
# # <<<< Dockerized-NorLab Project (end)

# Reload shell
source ~/.bashrc

# Remove repository (optional)
rm -rf /path/to/dockerized-norlab-project
```

## Advanced Installation Options

### Container-Based Installation

For use within containers or CI environments (docker in docker):

```dockerfile
FROM ubuntu:22.04

# Install dependencies
RUN apt-get update && apt-get install -y git docker.io

# Install DNA
RUN git clone --recurse-submodules https://github.com/norlab-ulaval/dockerized-norlab-project.git /opt/dna
RUN cd /opt/dna && bash install.bash --skip-system-wide-symlink-install
ENV PATH="/opt/dna/src/bin:$PATH"
```

### Network-Restricted Environments

For environments without internet access:

1. **Download on connected machine:**
   ```bash
   git clone --recurse-submodules https://github.com/norlab-ulaval/dockerized-norlab-project.git
   ```

2. **Transfer to target machine** and install:
   ```bash
   # Copy the DNA repository to the remote host
   scp -r dockerized-norlab-project user@remote-host:/path/to/destination/

   # SSH into the remote host and install
   ssh user@remote-host
   cd /path/to/destination/dockerized-norlab-project
   bash install.bash --skip-system-wide-symlink-install
   ```

## See Also

- [Getting Started Guide](../README.md#getting-started)
- [Project Initialization & Configuration](project_initialization_and_configuration.md)
- [Command Reference](dna.md)
- [Docker Installation Documentation](https://docs.docker.com/engine/install/)

## Navigation

- [← Back to Main README](../README.md)
- [Command Reference](dna.md)
- [Project Initialization & Configuration](project_initialization_and_configuration.md)
- [IDE Integration](ide_integration.md)
