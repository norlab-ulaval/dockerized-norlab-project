# IDE Integration

Complete guide to integrating DNA and Dockerized-NorLab (DN) with IDEs and development environments.

## Table of Contents

- [Overview](#overview)
- [PyCharm Integration](#pycharm-integration)
- [Visual Studio Code Integration](#visual-studio-code-integration)
- [Remote Development Workflows](#remote-development-workflows)
- [Debugging Setup](#debugging-setup)
- [Testing Integration](#testing-integration)
- [Common Development Patterns](#common-development-patterns)
- [Troubleshooting](#troubleshooting)

## Overview

DNA supports integration with various IDEs and development environments, enabling seamless containerized development workflows. This guide covers setup instructions for popular IDEs and development patterns.

## PyCharm Integration

### Prerequisites

- PyCharm Professional (required for remote development features)
- DNA project initialized with `dna init`
- Container built and running (`dna build develop && dna up`)

### Remote Development Setup

#### 1. Configure Remote Host Connection

1. **Open PyCharm** and go to `Settings | Build, Execution, Deployment | Deployment`

2. **Add new server** with the following settings:
   - **Type**: SFTP
   - **SSH configuration**:
     - **Name**: DNA Remote Development (or ant meaningfull name)
     - **Host**: `localhost` (for local containers) or remote machine IP
     - **Port**: `22` (for remote machines) or container SSH port
     - **Username**: Your remote username or container user
     - **Authentication**: Password or Key pair
   - **Root path**: Path to your project on remote/container
   - **Enable rsync**: Check "Use rsync for upload/download"
   - **Disable sudo**: Uncheck "sudo" option

3. **Configure rsync settings**:
   - Ensure `.git` directory is **not** in excluded paths
   - Add any project-specific exclusions

4. **Configure mapping**:
   - Go to the mapping tab
   - Set the `Deployment path` pointing to the remote host directory to sync with the `Local path` directory
5. **Test connection** and sync your project files

#### 3. Configure Remote Python Interpreter

1. **Go to** `Settings | Project: dockerized-norlab-pro... | Python Interpreter`

2. **Add new interpreter**:
   - Choose "SSH Interpreter"
   - **Host**: `localhost` (local) or remote IP
   - **Port**: Container SSH port (default: `2222`)
   - **Username**: Container user (from `SUPER_PROJECT_USER`)

3. **Important settings**:
   - ⚠️ **Uncheck** "Execute code with root privileges"
   - ⚠️ **Uncheck** "Automatically upload project files"
   - **Interpreter path**: `/usr/bin/python3` or specific Python path
   - **Sync folders**: Set to `/ros2_ws/src/YOUR_PROJECT_NAME` or delete entry

#### 4. **Add ROS to Python Path** 
Add ROS specific Python paths to interpreter

1. **Go to** `Settings | Project: dockerized-norlab-pro... | Python Interpreter` 
2. Click on the `Python interpreter` field, scroll to the bottom and click `Show all`
3. In the `Python interpreter` pop-up, click on the `sub-folder` icon
4. In the `Interpreter Paths` pop-up, add the following path base on your target settings:
   ```
   # Adjust paths based on your target ROS distribution 
   /opt/ros/humble/lib/python3.10/site-packages
   
   # Adjust paths based on your target Python version
   /ros2_ws/install/lib/python3.10/site-packages
   ```

#### 5. Configure Run/Debug Settings

1. **Create new run configuration**:
   - **Script path**: Path to your Python script
   - **Working directory**: `/ros2_ws/src/YOUR_PROJECT_NAME`

2. **Set path mappings**:
   - **Local path**: `~/PycharmProjects/YOUR_PROJECT_NAME`
   - **Remote path**: `/ros2_ws/src/YOUR_PROJECT_NAME`

3. **Environment variables**:
   - Load environment from: `.dockerized_norlab/dn_container_env_variable/.env.dn_expose_*`

### Container Environment Variables

To access container environment variables in PyCharm:

1. **Start your container** and attach to it:
   ```bash
   dna up
   dna attach
   ```

2. **Export environment variables**:
   ```bash
   dn_expose_container_env_variables
   ```

3. **Download the generated `.env` file** from:
   ```
   .dockerized_norlab/dn_container_env_variable/.env.dn_expose_CONTAINER_NAME
   ```

4. **Add to PyCharm run configuration**:
   - Go to run configuration settings
   - Set "Paths to .env files" to the downloaded file

### PyCharm Project Structure

Recommended PyCharm project structure:

```
PyCharm Project Root/
├── src/                    ← Your source code
├── tests/                  ← Test files
├── .dockerized_norlab/     ← DNA configuration
├── .idea/                  ← PyCharm settings
└── external_data/          ← Data files (excluded from sync)
```


## Remote Development Workflows

### Local Development with Remote Execution

**Scenario**: Edit locally, execute in remote container

1. **Setup file synchronization**:
   ```bash
   # Using rsync
   rsync -avz --exclude='.git' ./ user@remote:/path/to/project/
   ```

2. **Execute commands remotely**:
   ```bash
   # SSH into remote container
   ssh -p 2222 user@remote-host
   
   # Or use DNA attach
   dna attach
   ```

### Multi-Machine Development

**Scenario**: Team development across multiple machines

1. **Standardize environment**:
   ```bash
   # Each developer runs
   dna init
   dna build develop
   ```

2. **Share configuration**:
   - Commit `.dockerized_norlab/` to version control
   - Use `.env.local` for machine-specific settings

3. **Coordinate ports**:
   ```bash
   # Developer 1
   DN_SSH_SERVER_PORT=2222
   
   # Developer 2
   DN_SSH_SERVER_PORT=2223
   ```

## Debugging Setup

### Python Debugging

#### PyCharm Debugging

1. **Set breakpoints** in your Python code
2. **Create debug configuration**:
   - **Script path**: Your Python script
   - **Parameters**: Command line arguments
   - **Environment variables**: Load from container

3. **Start debugging** with remote interpreter

### C++ Debugging

#### Configure GDB for Container

1. **Install GDB** in container (usually pre-installed)
2. **Build with debug symbols**:
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
   ```

3. **Configure IDE** to use remote GDB

## Testing Integration

### PyTest Integration

#### PyCharm Setup

1. **Configure test runner**:
   - `File > Settings > Tools > Python Integrated Tools`
   - **Default test runner**: pytest

2. **Set test paths**:
   - **Test source folders**: `tests/`
   - **Working directory**: Project root

## Common Development Patterns

### Hot Reload Development

For rapid development cycles:

1. **Use develop mode** with mounted volumes:
   ```bash
   dna build develop
   dna up
   ```

2. **Edit files locally**, changes reflect immediately in container

3. **Restart services** as needed:
   ```bash
   # In container
   ros2 run your_package your_node
   ```

### Multi-Service Development

For complex projects with multiple services:

1. **Use docker-compose** for orchestration
2. **Configure IDE** to connect to specific services
3. **Use port forwarding** for service communication

### Cross-Platform Development

For projects targeting multiple architectures:

1. **Use multi-arch builds**:
   ```bash
   dna build --multiarch develop
   ```

2. **Test on target platforms**:
   ```bash
   # Build for ARM64 (Jetson)
   dna build --multiarch deploy
   ```

## Troubleshooting

### Common IDE Issues

#### PyCharm: "Cannot connect to remote host"

**Problem**: SSH connection fails.

**Solutions**:
1. Verify container is running: `dna up`
2. Check SSH port: `docker ps` and look for port mapping
3. Verify SSH service in container: `dna exec systemctl status ssh`

#### Debugging: "Breakpoints not hit"

**Problem**: Debugger doesn't stop at breakpoints.

**Solutions**:
1. Verify path mappings are correct
2. Check that debug symbols are available
3. Ensure source code matches running code

### Performance Issues

#### Slow File Synchronization

**Problem**: File sync between host and container is slow.

**Solutions**:
1. Use `.dockerignore` to exclude large files
2. Configure rsync exclusions
3. Use bind mounts for development

#### High CPU Usage

**Problem**: IDE uses excessive CPU.

**Solutions**:
1. Exclude large directories from indexing
2. Disable unnecessary plugins
3. Increase container resource limits

### Network Issues

#### Port Conflicts

**Problem**: IDE cannot connect due to port conflicts.

**Solutions**:
1. Change container ports in `.env.local`:
   ```bash
   DN_SSH_SERVER_PORT=2223
   DN_GDB_SERVER_PORT=7777
   ```

2. Update IDE connection settings accordingly

#### Firewall Blocking Connections

**Problem**: Firewall blocks IDE connections.

**Solutions**:
1. Configure firewall rules for container ports
2. Use SSH tunneling for remote connections
3. Check Docker network settings

## See Also

- [Project Initialization & Configuration](project_initialization_and_configuration.md) - Project setup
- [dna up](command/up.md) - Starting containers
- [dna attach](command/attach.md) - Connecting to containers
- [Installation Guide](install.md) - DNA installation

## Navigation

- [← Back to Main README](../README.md)
- [Command Reference](dna.md)
- [Installation Guide](install.md)
- [Project Initialization & Configuration](project_initialization_and_configuration.md)
