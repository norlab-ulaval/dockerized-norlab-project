# Offline Installation Guide

Complete guide for installing and using DNA in environments without internet connectivity.

## Overview

DNA supports offline installation scenarios common in:
- Air-gapped networks
- Remote field deployments  
- Secure environments with restricted internet access
- Development environments with limited connectivity

## Prerequisites

### Required Software (Must be Pre-installed)

Before attempting offline installation, ensure these tools are available:

| Tool           | Purpose | Installation Method |
|----------------|---------|-------------------|
| Docker Engine  | Container runtime | System package manager or Docker Desktop |
| Docker Compose | Multi-container orchestration | Included with Docker Desktop or separate install |
| Docker Buildx  | Multi-architecture build | Included with Docker Desktop or separate install |
| Git            | Version control | System package manager |
| Tree           | Directory visualization | System package manager |
| rsync          | Fast incremental file transfer | System package manager |

### Platform-Specific Prerequisites

#### Linux (Ubuntu/Debian)
```bash
# Install required packages
sudo apt-get update
sudo apt-get install -y docker.io docker-compose git tree

# Add user to docker group
sudo usermod -aG docker $USER
newgrp docker
```

#### macOS

1. Install Docker Desktop (manual download required).
See https://docs.docker.com/desktop/mac/install/
2. Install package manager and remaining required software
```bash
# Install Homebrew (if not present)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install tools
brew update && brew install git tree
```

#### NVIDIA Jetson (L4T)
```bash
sudo apt-get update && sudo apt-get install tree

# Most other tools come pre-installed, verify availability
docker --version
docker compose version
docker buildx version
git --version
```

## Offline Installation Process

### Step 1: Prepare DNA Repository (Online Machine)

```bash
# Clone with all submodules on connected machine
git clone --recurse-submodules https://github.com/norlab-ulaval/dockerized-norlab-project.git

# Create archive for transfer
tar -czf dna-offline.tar.gz dockerized-norlab-project/
```

### Step 2: Transfer to Offline Machine

```bash
# Transfer via USB, network share, or secure copy
scp dna-offline.tar.gz user@offline-machine:/tmp/

# Extract on offline machine
cd /tmp
tar -xzf dna-offline.tar.gz
cd dockerized-norlab-project
```

### Step 3: Install DNA Offline

```bash
# Run installation (automatically detects offline status)
bash install.bash

# Or use non-interactive mode
bash install.bash --yes
```

### Step 4: Verify Installation

```bash
# Test DNA availability
dna version

# Verify Docker integration
dna project validate
```

## Offline Limitations

### What Works Offline
- ✅ DNA command installation and path setup
- ✅ Project initialization (`dna init`)
- ✅ Local container builds (with pre-existing base images)
- ✅ Container lifecycle management
- ✅ Configuration management

### What Requires Internet
- ❌ Base image downloads
- ❌ Package manager updates
- ❌ Automatic software installation
- ❌ Docker registry operations

## Working with Pre-built Images

### Saving Images for Offline Use

On connected machine:
```bash
# Build and save DNA images
dna build develop
dna save --output dna-images.tar

# Transfer image archive to offline machine
```

On offline machine:
```bash
# Load pre-built images
dna load --input dna-images.tar

# Verify images are available
docker images
```

## Troubleshooting Offline Installation

### Common Issues

#### "Cannot connect to Docker daemon"
**Solution**: Ensure Docker service is running:
```bash
sudo systemctl start docker
sudo systemctl enable docker
```

#### "Package manager not found" (macOS)
**Solution**: Install Homebrew or MacPorts manually before running DNA installation.

#### "Base image not found"
**Solution**: Use pre-built images or modify Dockerfile to use locally available base images.

### Verification Commands

```bash
# Check all required tools
docker --version
docker compose version
docker buildx version  
git --version
dna version

# Test basic DNA functionality
cd /tmp
mkdir test-project && cd test-project
git init
dna init
```

## Best Practices

1. **Pre-stage Images**: Download and save all required Docker images before going offline
2. **Verify Prerequisites**: Ensure all required software is installed and working
3. **Test Installation**: Verify DNA works correctly before deploying to production
4. **Document Dependencies**: Keep track of any additional tools or images needed
5. **Regular Updates**: Periodically update DNA and images when connectivity is available

## See Also

- [← Back to Main README](../README.md)
- [Installation Guide](install.md)
- [Command Reference](dna.md)
- [Project Configuration](project_initialization_and_configuration.md)
