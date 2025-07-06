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
├── .dockerized_norlab/                 ← DNA configuration directory
│   ├── configuration/                  ← Main configuration files
│   │   ├── project_entrypoints/        ← Container startup scripts
│   │   ├── project_requirements/       ← Dependency specifications
│   │   ├── Dockerfile                  ← Container build instructions
│   │   ├── .env.dna                    ← DNA-specific env variables
│   │   ├── .env                        ← Project-specific env variables
│   │   ├── .env.local                  ← Local env variables overrides
│   │   └── README.md                   ← Configuration documentation
│   ├── dn_container_env_variable/      ← Container environment exports
│   ├── .env.your-project-repository    ← Project DNA configuration meta
│   └── README.md                       ← DNA configuration quick documentation
├── artifact/                           ← Runtime produced data (mounted)
├── external_data/                      ← Pre-existing data (mounted)
├── src/                                ← Your source code (mounted/copied)
├── tests/                              ← Your test code (mounted/copied)
...
├── .dockerignore                       ← Docker build exclusions
├── .gitignore                          ← Git exclusions
└── README.md                           ← Project documentation
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
2. `.env` (super project) - General project settings
3. `.env.local` (super project) - Local env variables overrides
4. `.env.dna-internal` (DNA repo) - Internal DNA settings

#### `.env` - Project-specific Environment Variables

Main project configuration file. This file is for project related environment variable (i.e., non-DNA/DN env var).

#### `.env.dna` - DNA-specific environment variables
This file is for DN/DNA specific setting.
Variables `DN_PROJECT_GIT_REMOTE_URL`, `DN_CONTAINER_NAME` and `DN_PROJECT_ALIAS_PREFIX` are automaticaly configured on initialization.
Check `.env.dna` comment for other available environment variable.


#### `.env.local` - Local environment variables overrides

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

There is three method for configuring container in DNA. In execution order:
1. via `pip` using python requirement file `python.requirements.txt`
2. using shell script file `shell.requirements.bash`
3. using the `Dockerfile` stage `user-project-custom-steps` (see [Docker Configuration](#docker-configuration) for details)

Each one of them serve different purposes. Use the ones best suited for your project needs. 
You can use all tree in combinaison if necessary. 

### Specifying Python Requirements

Specify Python dependencies in `.dockerized_norlab/configuration/project_requirements/python.requirements.txt`:

Example:
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
Documentation
 - Requirements File Format:  https://pip.pypa.io/en/stable/reference/requirements-file-format/
 - Requirement Specifiers:  https://pip.pypa.io/en/stable/reference/requirement-specifiers/


### Shell Requirements

Specify shell dependencies in `.dockerized_norlab/configuration/project_requirements/shell.requirements.bash` as if it is a instalation script.  


## Project Entrypoints

Files in `.dockerized_norlab/configuration/project_entrypoints` are customizable callback script executed by the docker container entrypoint. Each one of them serve different purposes:
- `dn_entrypoint.global.*.callback.bash` are executed in all mode (develop, deploy, ci-tests and slurm) 
- `<mode>/dn_entrypoint.*.callback.bash` are specialized version executed after the global one and only in that mode
- `*.init.callback.bash` are executed on container initialization only. It happen only once in a container life-cycle.
- `*.attach.callback.bash` are executed on every time a shell is attach to a conatiner. It can happen many time in a container life-cycle.  

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

**Solution**: Ensure `DN_PROJECT_USER` matches host user:
```bash
# Host user
id -un

# DN project container user
dna project dotenv | grep -e DN_PROJECT_USER -e DN_PROJECT_UID -e DN_PROJECT_GID
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
