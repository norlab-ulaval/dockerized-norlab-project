# Project Initialization & Configuration

Complete guide to setting up and configuring DNA projects for containerized robotic development.

## Table of Contents

- [Overview](#overview)
- [Project Initialization](#project-initialization)
- [Directory Structure](#directory-structure)
- [Configuration Files](#configuration-files)
- [Environment Variables](#environment-variables)
- [Docker Configuration](#docker-configuration)
- [Project Requirements](#project-requirements)
- [Project Entrypoints](#project-entrypoints)
- [Customization Examples](#customization-examples)
- [Best Practices](#best-practices)
- [Troubleshooting](#troubleshooting)

## Overview

DNA transforms regular Git repositories into containerized development environments through the `dna init` command. This process creates a standardized directory structure and configuration files that enable reproducible, isolated development workflows.

## Project Initialization

### Prerequisites

Before initializing a DNA project:

1. **Git repository**: Your project must be under Git version control
2. **Repository root**: Run commands from the repository root directory
3. **DNA installed**: Ensure DNA is properly installed on your system

### Basic Initialization

```bash
# Navigate to your project repository
cd /path/to/your/project

# Initialize DNA
dna init
```

### What Happens During Initialization

1. **Validation**: Checks for Git repository and proper location
2. **Directory creation**: Creates necessary directories and files
3. **Template copying**: Copies configuration templates
4. **Customization**: Adapts templates to your project
5. **Preservation**: Updates existing files without overwriting

## Directory Structure

After running `dna init`, your project will have the following structure:

```
your-project-repository/
├── .dockerized_norlab/             ← DNA configuration directory
│   ├── configuration/              ← Main configuration files
│   │   ├── .env                    ← Project environment variables
│   │   ├── .env.dna                ← DNA-specific variables
│   │   ├── .env.local              ← Local development overrides
│   │   ├── Dockerfile              ← Container build instructions
│   │   ├── README.md               ← Configuration documentation
│   │   ├── project_entrypoints/    ← Container startup scripts
│   │   └── project_requirements/   ← Dependency specifications
│   ├── dn_container_env_variable/  ← Container environment exports
│   └── visual/                     ← Project-specific visuals
├── artifact/                       ← Runtime produced data (mounted)
├── external_data/                  ← Pre-existing data (mounted)
├── src/                           ← Your source code (mounted/copied)
├── tests/                         ← Your test code (mounted/copied)
├── .dockerignore                  ← Docker build exclusions
├── .gitignore                     ← Git exclusions
└── README.md                      ← Project documentation
```

### Directory Purposes

| Directory | Purpose | Mount Behavior |
|-----------|---------|----------------|
| `src/` | Source code | Mounted (develop) / Copied (deploy) |
| `tests/` | Test code | Mounted (develop) / Copied (deploy) |
| `artifact/` | Runtime data | Persistent volume mount |
| `external_data/` | External datasets | Read-only mount |
| `.dockerized_norlab/` | DNA configuration | Build context only |

## Configuration Files

### Environment Files

DNA uses a hierarchical environment variable system with the following precedence:

1. `.env.dna` (super project) - DNA-specific settings
2. `.env.dna-internal` (DNA repo) - Internal DNA settings
3. `.env` (super project) - General project settings
4. `.env.local` (super project) - Local development overrides

#### `.env` - Project Environment Variables

Main project configuration file:

```bash
# Project identification
SUPER_PROJECT_NAME=my-robot-project
SUPER_PROJECT_USER=developer

# ROS configuration
ROS_DISTRO=humble
ROS_DOMAIN_ID=42

# Container configuration
CONTAINER_NAME=my-robot-dev
CONTAINER_HOSTNAME=robot-dev

# Network configuration
CONTAINER_SSH_PORT=2222
CONTAINER_VNC_PORT=5901
```

#### `.env.dna` - DNA-Specific Variables

DNA application settings:

```bash
# Build configuration
DNA_BUILD_CONTEXT=.
DNA_IMAGE_TAG=latest
DNA_DOCKERFILE_PATH=.dockerized_norlab/configuration/Dockerfile

# Service configuration
DNA_SERVICE_DEVELOP=develop
DNA_SERVICE_DEPLOY=deploy
DNA_SERVICE_CI_TESTS=ci-tests

# Registry configuration
DNA_REGISTRY=docker.io
DNA_NAMESPACE=myorganization
```

#### `.env.local` - Local Development Overrides

Local-specific settings (not committed to Git):

```bash
# Local development ports
CONTAINER_SSH_PORT=2223
CONTAINER_VNC_PORT=5902

# Local paths
EXTERNAL_DATA_PATH=/home/user/datasets
ARTIFACT_PATH=/home/user/artifacts

# Development flags
DEBUG_MODE=true
VERBOSE_LOGGING=true
```

### Environment Variable Reference

#### Core Variables

| Variable | Description | Example |
|----------|-------------|---------|
| `SUPER_PROJECT_NAME` | Project identifier | `my-robot-project` |
| `SUPER_PROJECT_USER` | Container user | `developer` |
| `ROS_DISTRO` | ROS distribution | `humble`, `iron`, `rolling` |
| `CONTAINER_NAME` | Container name | `my-robot-dev` |

#### Network Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `CONTAINER_SSH_PORT` | SSH access port | `2222` |
| `CONTAINER_VNC_PORT` | VNC access port | `5901` |
| `CONTAINER_JUPYTER_PORT` | Jupyter port | `8888` |

#### Path Variables

| Variable | Description | Default |
|----------|-------------|---------|
| `ARTIFACT_PATH` | Host artifact path | `./artifact` |
| `EXTERNAL_DATA_PATH` | Host data path | `./external_data` |
| `SRC_PATH` | Host source path | `./src` |

## Docker Configuration

### Dockerfile Customization

The generated `Dockerfile` can be customized for your specific needs:

```dockerfile
# Base image selection
ARG BASE_IMAGE=norlabulaval/dn-project-core:humble-l4t

FROM ${BASE_IMAGE} AS develop-stage

# Install project-specific dependencies
RUN apt-get update && apt-get install -y \
    python3-opencv \
    ros-${ROS_DISTRO}-navigation2 \
    && rm -rf /var/lib/apt/lists/*

# Install Python packages
COPY project_requirements/requirements.txt /tmp/
RUN pip3 install -r /tmp/requirements.txt

# Copy project source
COPY src/ /ros2_ws/src/${SUPER_PROJECT_NAME}/

# Build ROS workspace
RUN cd /ros2_ws && \
    source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select ${SUPER_PROJECT_NAME}

# Set up entrypoint
COPY project_entrypoints/ /dockerized_norlab_entrypoints/
RUN chmod +x /dockerized_norlab_entrypoints/*.bash

ENTRYPOINT ["/dockerized_norlab_entrypoints/entrypoint.bash"]
```

### Multi-Stage Build Example

```dockerfile
# Development stage
FROM norlabulaval/dn-project-core:humble AS develop-stage
# ... development-specific setup

# Deploy stage
FROM develop-stage AS deploy-stage
# Remove development tools
RUN apt-get remove -y \
    build-essential \
    cmake \
    && apt-get autoremove -y

# Production optimizations
RUN rm -rf /var/lib/apt/lists/* \
    && rm -rf /tmp/*
```

## Project Requirements

### Python Requirements

Specify Python dependencies in `project_requirements/requirements.txt`:

```txt
# Core dependencies
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.4.0

# ROS-specific packages
rclpy
geometry_msgs
sensor_msgs

# Development tools (for develop stage only)
pytest>=6.0.0
black>=21.0.0
flake8>=3.9.0
```

### System Requirements

Specify system packages in `project_requirements/apt_packages.txt`:

```txt
# System dependencies
libeigen3-dev
libpcl-dev
libopencv-dev

# ROS packages
ros-humble-navigation2
ros-humble-slam-toolbox
ros-humble-robot-localization
```

### ROS Dependencies

Specify ROS dependencies in `project_requirements/rosdep.yaml`:

```yaml
# ROS package dependencies
dependencies:
  - geometry_msgs
  - sensor_msgs
  - nav_msgs
  - tf2
  - tf2_ros
  - rclcpp
  - rclpy
```

## Project Entrypoints

### Main Entrypoint

The main container entrypoint (`project_entrypoints/entrypoint.bash`):

```bash
#!/bin/bash
# Main container entrypoint

# Source ROS environment
source /opt/ros/${ROS_DISTRO}/setup.bash

# Source project workspace
if [ -f "/ros2_ws/install/setup.bash" ]; then
    source /ros2_ws/install/setup.bash
fi

# Set up environment
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=rmw_cyclonedx_cpp

# Start SSH daemon for remote access
sudo service ssh start

# Execute command or start interactive shell
if [ $# -eq 0 ]; then
    exec /bin/bash
else
    exec "$@"
fi
```

### Development Entrypoint

Development-specific setup (`project_entrypoints/develop_entrypoint.bash`):

```bash
#!/bin/bash
# Development entrypoint with additional tools

# Source main entrypoint
source /dockerized_norlab_entrypoints/entrypoint.bash

# Development environment setup
export PYTHONPATH="/ros2_ws/src:$PYTHONPATH"
export AMENT_PREFIX_PATH="/ros2_ws/install:$AMENT_PREFIX_PATH"

# Start development services
if [ "${START_JUPYTER:-false}" = "true" ]; then
    jupyter lab --ip=0.0.0.0 --port=8888 --no-browser --allow-root &
fi

if [ "${START_VNC:-false}" = "true" ]; then
    vncserver :1 -geometry 1920x1080 -depth 24 &
fi

# Continue with main entrypoint
exec "$@"
```

## Customization Examples

### Adding Custom Dependencies

1. **Edit requirements file:**
   ```bash
   echo "opencv-python>=4.5.0" >> .dockerized_norlab/configuration/project_requirements/requirements.txt
   ```

2. **Rebuild container:**
   ```bash
   dna build develop
   ```

### Custom Environment Variables

1. **Add to `.env`:**
   ```bash
   echo "CUSTOM_DATASET_PATH=/data/custom" >> .dockerized_norlab/configuration/.env
   ```

2. **Use in Dockerfile:**
   ```dockerfile
   ENV CUSTOM_DATASET_PATH=${CUSTOM_DATASET_PATH}
   ```

### Multi-Robot Configuration

For projects with multiple robots:

```bash
# .env.robot1
SUPER_PROJECT_NAME=multi-robot-project
ROBOT_ID=robot1
CONTAINER_NAME=robot1-dev
CONTAINER_SSH_PORT=2222
ROS_DOMAIN_ID=1

# .env.robot2
SUPER_PROJECT_NAME=multi-robot-project
ROBOT_ID=robot2
CONTAINER_NAME=robot2-dev
CONTAINER_SSH_PORT=2223
ROS_DOMAIN_ID=2
```

## Best Practices

### Environment Management

1. **Use `.env.local` for sensitive data:**
   ```bash
   # Don't commit API keys or passwords
   API_KEY=your-secret-key
   DATABASE_PASSWORD=secret
   ```

2. **Document environment variables:**
   ```bash
   # .env.example
   SUPER_PROJECT_NAME=your-project-name
   ROS_DISTRO=humble
   # Add your custom variables here
   ```

### Docker Optimization

1. **Use multi-stage builds** for smaller production images
2. **Minimize layers** by combining RUN commands
3. **Use .dockerignore** to exclude unnecessary files
4. **Pin dependency versions** for reproducibility

### Development Workflow

1. **Use develop mode** for active development:
   ```bash
   dna build develop
   dna up
   ```

2. **Test with deploy mode** before production:
   ```bash
   dna build deploy
   dna run deploy your-test-command
   ```

3. **Use CI mode** for automated testing:
   ```bash
   dna build ci-tests
   dna run ci-tests pytest
   ```

## Troubleshooting

### Common Configuration Issues

#### Environment Variables Not Loading

**Problem**: Custom environment variables not available in container.

**Solutions:**
1. Check file precedence order
2. Verify syntax (no spaces around `=`)
3. Rebuild container after changes

#### Port Conflicts

**Problem**: Container ports already in use.

**Solution**: Change ports in `.env.local`:
```bash
CONTAINER_SSH_PORT=2223
CONTAINER_VNC_PORT=5902
```

#### Permission Issues

**Problem**: Files created in container have wrong ownership.

**Solution**: Ensure `SUPER_PROJECT_USER` matches host user:
```bash
SUPER_PROJECT_USER=$(id -un)
```

#### Build Failures

**Problem**: Docker build fails with dependency errors.

**Solutions:**
1. Check internet connectivity
2. Verify package names in requirements files
3. Update base image versions

### Configuration Validation

Check your configuration:

```bash
# Validate environment files
dna project dotenv

# Check project structure
dna project validate

# Run sanity checks
dna project sanity
```

## See Also

- [dna init](command/init.md) - Initialize DNA projects
- [dna build](command/build.md) - Build container images
- [dna project](command/project.md) - Project management commands
- [Installation Guide](install.md) - DNA installation
- [IDE Integration](ide_integration.md) - Development environment setup

## Navigation

- [← Back to Main README](../README.md)
- [Command Reference](dna.md)
- [Installation Guide](install.md)
- [IDE Integration](ide_integration.md)
