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

Main project configuration file. This file is for project related environment variable (i.e., non-DNA/DN env var).

#### `.env.dna` - DNA-Specific Variables
This file is for DN/DNA specific setting.
Variables `DN_PROJECT_GIT_REMOTE_URL`, `DN_CONTAINER_NAME` and `DN_PROJECT_ALIAS_PREFIX` are automaticaly configured on initialization.
Check `.env.dna` comment for other available environment variable.


#### `.env.local` - Local Development Overrides

This file won't be committed to Git.
Use it for local-specific settings.
Example:

```bash
# Local development ports
DN_SSH_SERVER_PORT=2223
DN_GDB_SERVER_PORT=7777

# Development flags
DEBUG_MODE=true
VERBOSE_LOGGING=true
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


## Docker Configuration

### Dockerfile Customization

The generated `.dockerized_norlab/configuration/Dockerfile` can be customized for your specific needs. 
In most cases however, using only `python.requirements.txt` and/or `shell.requirements.bash` is enough.

#### ⚠️ Do not add code to the first stage `init-and-setup` unless you know what your doing.
```dockerfile
# =================================================================================================
#
# Usage:
#   - 👍 You can change code in the 'user-project-custom-steps' stage.
#     See the line with the "↓ ↓ ↓ ..." character below.
#   - ⚠️ Dont change code the first stage (init-and-setup) or in the last stage (final) unless you
#     know what your doing.
#
# =================================================================================================
ARG BASE_IMAGE
ARG BASE_IMAGE_TAG
FROM ${BASE_IMAGE:?err}:${BASE_IMAGE_TAG:?err} AS init-and-setup

# ...
```

#### User can add code in the `user-project-custom-steps` stage.
Check the following lines in `.dockerized_norlab/configuration/Dockerfile`
```dockerfile
# ...

# ====User project custom steps====================================================================
FROM init-and-setup AS user-project-custom-steps
# USER NOTES: ADD YOUR CODE IN THIS STAGE
# ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
# ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓

# Example
RUN <<EOF
    {
        echo "..........................................." && \
        echo "Sanity check" && \
        python -c "import torch" && \
        python -c "import torchvision" && \
        python -c "import hydra" && \
        python -c "from omegaconf import DictConfig, OmegaConf" && \
        echo "..........................................." ;
    } || exit 1
EOF

# ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑
# ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑ ↑
# USER NOTES: ADD YOUR CODE BEFORE THIS LINE

# ...
```
#### Expert user can also add code to the `final` stage for handling special cases.
Check the following lines in `.dockerized_norlab/configuration/Dockerfile`

```dockerfile
# ...

# ====DN-project final=============================================================================
FROM --platform=${TARGETPLATFORM} user-project-custom-steps AS final
# ⚠️ USER NOTES: Dont change code in this stage unless you know what your doing.
ARG TARGETPLATFORM
ARG BUILDPLATFORM
WORKDIR ${DN_PROJECT_PATH:?'environment variable is not set'}

RUN <<EOF
    source /dna-lib-container-tools/dn_project_core.build.aarch_aware_build_ros.bash ${TARGETPLATFORM} ${BUILDPLATFORM} ${DN_DEV_WORKSPACE:?err}/src || exit 1
    # Cleanup buidl script
    rm -f /dn_project_core_init.bash
    rm -f /dna-lib-container-tools/dn_project_core.setup.bash
    rm -f /dna-lib-container-tools/dn_project_core.build.aarch_aware_build_ros.bash
EOF
CMD [ "bash" ]
```

---

## Best Practices

### Environment Management

1. **Use `.env.local` for sensitive data:**
   ```bash
   # Don't commit API keys or passwords
   API_KEY=your-secret-key
   DATABASE_PASSWORD=secret
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
DN_SSH_SERVER_PORT=2223
DN_GDB_SERVER_PORT=7778
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
