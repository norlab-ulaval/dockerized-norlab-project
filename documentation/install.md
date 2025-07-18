# Installation Guide

Complete installation instructions for Dockerized-NorLab project application on all supported platforms.

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

- **Internet connection**: Required for initial setup and image downloads (offline installation supported with limitations)
- **Git**: Version control system
- ***tree***: List directory content in a tree-like format in terminal

#### Operating system specific: L4T and Ubuntu 

- **Docker Engine**: Version 20.10 or later with BuildKit support
- **Docker Compose**: Plugin version (v2) or standalone
- **Docker Buildx**: For multi-architecture builds

#### Operating system specific: macOs 

- **Docker Desktop**: Comme with docker engine, docker compose and docker buildx pre-installed
- **macOS Package Manager**: Homebrew or MacPorts (automatically installed if missing on macOS)

### Optional Requirements

- **NVIDIA Docker**: Required for GPU acceleration support
- **[Dockerhub account](https://docs.docker.com/accounts/create-account/**: Required for online build, sharing deploy image online, and publishing release image


### Platform Support

| Platform       | Architecture | Status                                         |
|----------------|------------|------------------------------------------------|
| NVIDIA Jetson (L4T) | arm64 | ✅ Fully supported                              |
| macOS          | arm64 (Apple Silicon) | ✅ Fully supported                              |
| Linux          | x86_64 | ✅ Fully supported                              |
| Linux          | arm64 | ⚠️ Not supported (for now, will be implemented if requested) |


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
- Sudo privileges for symlink creation and directory creation (if needed)
- If `/usr/local/bin` doesn't exist, the installer will prompt to create it

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

## Offline Installation

DNA supports installation in environments without internet access, with some limitations.

### Offline Installation Process

When running `install.bash` without internet connectivity:

1. **Automatic Detection**: The installer detects offline status and adapts accordingly
2. **Software Requirements**: Installation of Docker, Git, and other tools is skipped with warnings
3. **Path Configuration**: DNA path setup and symlink creation proceed normally
4. **User Notification**: Clear warnings about skipped steps and manual requirements

### Manual Requirements for Offline Installation

When installing offline, ensure these requirements are manually satisfied:

- **Docker Engine**: Must be pre-installed
- **Docker Compose**: Must be pre-installed  
- **Git**: Must be pre-installed
- **Tree**: Optional but recommended for directory visualization
- **Docker Buildx**: Required for multi-architecture builds

### Offline Installation Verification

```bash
# Verify required tools are available
docker --version
docker compose version
docker buildx version
git --version

# Test DNA installation
dna version
```

**See also [Offline Installation Guide](offline_installation.md) for manual install instructions**

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

   **Note**: The installer will automatically:
   - Detect if Homebrew or MacPorts is installed
   - Offer to install Homebrew if neither is present
   - Install required packages (`git`, `tree`) using the detected package manager
   - Skip software installation if offline (with appropriate warnings)

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
docker buildx create --name local-builder-multiarch-virtual --driver=docker-container --driver-opt="default-load=true" --platform linux/amd64,linux/arm64 --bootstrap --buildkitd-flags '--allow-insecure-entitlement network.host'
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
docker buildx version

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
1. **Re-run installer** (handles missing directory automatically):
   ```bash
   bash install.bash
   ```
   The installer will prompt to create `/usr/local/bin` if it doesn't exist.

2. **Use non-interactive installation**:
   ```bash
   bash install.bash --yes
   ```

3. **Use alternative installation**:
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

**Problem**: Received error `fatal: detected dubious ownership`.
```shell
$ bash install.bash 
fatal: detected dubious ownership in repository at '/opt/dockerized-norlab-project'
To add an exception for this directory, call:

        git config --global --add safe.directory /opt/dockerized-norlab-project
```

**Solution 1 (system level)**: Add repository and submodule to git config safe directory:

This error may happen when the system wide git config file `${prefix}/etc/gitconfig` prefix is set to an arbitrary
location instead of root. 

```shell
DNA_INSTALL_PATH="/opt/dockerized-norlab-project"
GIT_CONFIG_SCOPE="--system"
sudo git config --system --add safe.directory "${DNA_INSTALL_PATH}" && \
    sudo git config --system --add safe.directory "${DNA_INSTALL_PATH}/utilities/norlab-shell-script-tools" && \
    sudo git config --system --add safe.directory "${DNA_INSTALL_PATH}/utilities/norlab-build-system" && \
    sudo git config --system --add safe.directory "${DNA_INSTALL_PATH}/utilities/norlab-build-system/utilities/norlab-shell-script-tools"
```
Note:
- Change `DNA_INSTALL_PATH` to your `dockerized-norlab-project` directory location;
- Change `GIT_CONFIG_SCOPE` to:
  - `--system` for sytem wide git config -> recommended for server installation;
  - `--file PATH` for setting git config in a arbitrary system level git config file.

**Solution 2 (arbitrary scope)**: Add repository and submodule to git config safe directory:


```shell
DNA_INSTALL_PATH="/opt/dockerized-norlab-project"
GIT_CONFIG_SCOPE="--global" 
git config "${GIT_CONFIG_SCOPE}" --add safe.directory "${DNA_INSTALL_PATH}" && \
    git config "${GIT_CONFIG_SCOPE}" --add safe.directory "${DNA_INSTALL_PATH}/utilities/norlab-shell-script-tools" && \
    git config "${GIT_CONFIG_SCOPE}" --add safe.directory "${DNA_INSTALL_PATH}/utilities/norlab-build-system" && \
    git config "${GIT_CONFIG_SCOPE}" --add safe.directory "${DNA_INSTALL_PATH}/utilities/norlab-build-system/utilities/norlab-shell-script-tools"
```
Note:
- Change `DNA_INSTALL_PATH` to your `dockerized-norlab-project` directory location;
- Change `GIT_CONFIG_SCOPE` to:
  - `--global` for setting git config globally -> recommended for macOs;
  - `--local` for setting git config at repository level -> require priviledge if root own the DNA install directory.
  - `--file PATH` for setting git config in a arbitrary git config file.



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

#### Offline Installation Issues

**Problem**: Installation fails or shows warnings about missing software.

**Solution**: 
1. **Verify offline detection**:
   ```bash
   # Test connectivity on a reliable dns adresse (google), should fail if truly offline
   ping -c 1 8.8.8.8
   ```

2. **Manual software installation**: Install required tools manually:
   - **Linux**: Use system package manager (`apt`, `yum`, etc.)
   - **macOS**: Install Docker Desktop and use Homebrew/MacPorts for other tools
   - **Jetson**: Most tools come pre-installed

3. **Re-run installation**: After manual setup:
   ```bash
   bash install.bash --skip-system-wide-symlink-install
   ```

#### macOS Package Manager Issues

**Problem**: Neither Homebrew nor MacPorts is installed, and automatic installation fails.

**Solutions**:
- **Use non-interactive installation**:
   ```bash
   bash install.bash --yes  # Automatically installs Homebrew if missing
   ```
- **Install package manager manually**:
    - For **Homebrew**:
       ```bash
       /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
       ```
    - For **MacPorts**: Download from [MacPorts website](https://ports.macports.org)

**Problem**: Package installation fails with Homebrew or MacPorts.

**Solution**: Update package manager and retry:
```bash
# For Homebrew
brew update && brew doctor

# For MacPorts  
sudo port selfupdate
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
# # >>>> dockerized-norlab-project (start)
# export _DNA_PATH="/path/to/dna/bin"
# export PATH="$PATH:$_DNA_PATH"
# # <<<< dockerized-norlab-project (end)

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
- [Offline Installation Guide](offline_installation.md)
- [Project Initialization & Configuration](project_initialization_and_configuration.md)
- [Command Reference](dna.md)
- [Docker Installation Documentation](https://docs.docker.com/engine/install/)

## Navigation

- [← Back to Main README](../README.md)
- [Command Reference](dna.md)
- [Project Initialization & Configuration](project_initialization_and_configuration.md)
- [IDE Integration](ide_integration.md)
