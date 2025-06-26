# Documentation Updates Proposal

Based on the diff between the current branch (`fix-add-missing-macosx-compatible-install-logic-NMO-729`) and the `dev` branch, the following documentation updates are proposed to reflect the new macOS compatibility and offline installation features.

## Summary of Changes

The branch introduces significant improvements to the installation process:

1. **Enhanced macOS Compatibility**: Automatic detection and installation of Homebrew/MacPorts
2. **Offline Installation Support**: Graceful handling of offline scenarios with appropriate warnings
3. **Modular Installation Functions**: Refactored `setup_host_dna_requirements.bash` into separate, testable functions
4. **Improved Error Handling**: Better user feedback and non-blocking warnings
5. **Comprehensive Test Coverage**: New test cases for offline and macOS scenarios

## Proposed Documentation Updates

### 1. README.md Updates

#### Section: "Getting started" - Install section (Lines 192-204)

**Current:**
```shell
# Clone repository on host computer
git clone --recurse-submodule https://github.com/norlab-ulaval/dockerized-norlab-project.git
cd dockerized-norlab-project

# Install DNA on host (Check install option with $ bash install.bash --help) 
bash install.bash

# Check available commands
dna 
```

**Proposed:**
```shell
# Clone repository on host computer
git clone --recurse-submodule https://github.com/norlab-ulaval/dockerized-norlab-project.git
cd dockerized-norlab-project

# Install DNA on host (Check install option with $ bash install.bash --help) 
# Note: Installation supports both online and offline scenarios
bash install.bash

# Check available commands
dna 
```

### 2. documentation/install.md Updates

#### Section: "System Requirements" (Lines 23-28)
**Current:**
```markdown
### Core Requirements

- **Git**: Version control system
- **Docker Engine**: Version 20.10 or later with BuildKit support
- **Docker Compose**: Plugin version (v2) or standalone
- **Internet connection**: Required for initial setup and image downloads
```

**Proposed:**
```markdown
### Core Requirements (L4T and Ubuntu)

- **Docker Engine**: Version 20.10 or later with BuildKit support
- **Docker Compose**: Plugin version (v2) or standalone

### Core Requirements (macOs)

- **Docker Desktop**: Comme with docker engine, docker compose and docker buildx pre-installed
- **macOS Package Manager**: Homebrew or MacPorts (automatically installed if missing on macOS)

### Core Requirements (All)

- **Internet connection**: Required for initial setup and image downloads (offline installation supported with limitations)
- **Git**: Version control system
- ***tree***: List directory content in a tree-like format in terminal

```

#### Section: "Platform-Specific Instructions" - macOS (Lines 148-161)

**Current:**
````markdown
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
````

3. Follow post-installation setup instructions

**Proposed:**
````markdown
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
````

3. Follow post-installation setup instructions

#### New Section: "Offline Installation" (Insert after "Installation Methods")
**Add new section:**
````markdown
## Offline Installation

DNA supports installation in environments without internet access, with some limitations.

### Offline Installation Process

When running `install.bash` without internet connectivity:

1. **Automatic Detection**: The installer detects offline status and adapts accordingly
2. **Software Requirements**: Installation of Docker, Git, and other tools is skipped with warnings
3. **Path Configuration**: DNA path setup and symlink creation proceed normally
4. **User Notification**: Clear warnings about skipped steps and manual requirements

### Offline Installation Example

```bash
# Clone repository (must be done on connected machine)
git clone --recurse-submodules https://github.com/norlab-ulaval/dockerized-norlab-project.git

# Transfer to offline machine and install
cd dockerized-norlab-project
bash install.bash  # Will detect offline status automatically
```

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
dna --help
```
````

#### Section: "Troubleshooting" - Add new subsection (After line 309)
**Add new troubleshooting section:**
````markdown
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
````

### 3. New Documentation File: documentation/offline_installation.md

**Create new comprehensive offline installation guide:**
````markdown
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

| Tool | Purpose | Installation Method |
|------|---------|-------------------|
| Docker Engine | Container runtime | System package manager or Docker Desktop |
| Docker Compose | Multi-container orchestration | Included with Docker Desktop or separate install |
| Git | Version control | System package manager |
| Tree | Directory visualization | System package manager |

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
```bash
# Install Docker Desktop (manual download required)
# Install Homebrew (if not present)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install tools
brew install git tree
```

#### NVIDIA Jetson (L4T)
```bash
# Most tools come pre-installed, verify availability
docker --version
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
dna --help

# Verify Docker integration
dna config
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
````

**Add navigation link pointing to `offline_installation.md`**
- Add the navigation link in `README.md`
- Add the navigation link in `documentation/install.md`

## Implementation Priority

1. **High Priority**: README.md and install.md updates (core user-facing documentation)
2. **Medium Priority**: New offline installation guide (comprehensive reference)
3. **Low Priority**: Additional troubleshooting sections (edge cases)

## Testing Recommendations

After implementing these documentation updates:

1. **Review with stakeholders**: Ensure accuracy and completeness
2. **Test installation scenarios**: Verify instructions work for both online and offline cases
3. **Validate macOS instructions**: Test on actual macOS systems with and without package managers
4. **Check cross-references**: Ensure all internal links remain valid

This proposal addresses the key improvements introduced in the branch while maintaining consistency with existing documentation structure and style.
