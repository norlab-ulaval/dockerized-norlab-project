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

1. **Open PyCharm** and go to `File > Settings > Build, Execution, Deployment > Deployment`

2. **Add new server** with the following settings:
   - **Type**: SFTP
   - **Name**: DNA Remote Development
   - **Host**: `localhost` (for local containers) or remote machine IP
   - **Port**: `22` (for remote machines) or container SSH port
   - **Username**: Your remote username or container user

3. **Configure connection**:
   - **Authentication**: Password or Key pair
   - **Root path**: Path to your project on remote/container

#### 2. Setup Rsync Synchronization

1. **Enable rsync** in deployment settings:
   - Check "Use rsync for upload/download"
   - Uncheck "sudo" option

2. **Configure rsync options**:
   - Ensure `.git` directory is **not** in excluded paths
   - Add any project-specific exclusions

3. **Test connection** and sync your project files

#### 3. Configure Remote Python Interpreter

1. **Go to** `File > Settings > Project > Python Interpreter`

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

#### 4. Add ROS Python Paths

Add ROS-specific Python paths to interpreter:

1. **Go to interpreter settings** and click the gear icon
2. **Show paths** and add:
   ```
   /opt/ros/humble/lib/python3.10/site-packages
   /ros2_ws/install/lib/python3.10/site-packages
   ```
   (Adjust paths based on your ROS distribution and Python version)

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

## Visual Studio Code Integration

### Prerequisites

- Visual Studio Code
- Remote Development extension pack
- Docker extension

### Remote Container Development

#### 1. Install Required Extensions

```bash
# Install VS Code extensions
code --install-extension ms-vscode-remote.remote-containers
code --install-extension ms-vscode-remote.remote-ssh
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
```

#### 2. Configure Dev Container

Create `.vscode/devcontainer.json`:

```json
{
    "name": "DNA Development Container",
    "dockerComposeFile": "../.dockerized_norlab/configuration/docker-compose.yml",
    "service": "develop",
    "workspaceFolder": "/ros2_ws/src/${localEnv:PROJECT_NAME}",
    "shutdownAction": "stopCompose",
    
    "customizations": {
        "vscode": {
            "extensions": [
                "ms-python.python",
                "ms-vscode.cpptools",
                "ms-iot.vscode-ros",
                "twxs.cmake",
                "ms-python.flake8"
            ],
            "settings": {
                "python.defaultInterpreterPath": "/usr/bin/python3",
                "python.linting.enabled": true,
                "python.linting.flake8Enabled": true,
                "ros.distro": "humble"
            }
        }
    },
    
    "forwardPorts": [2222, 5901, 8888],
    "postCreateCommand": "source /opt/ros/humble/setup.bash",
    
    "remoteUser": "${localEnv:SUPER_PROJECT_USER}"
}
```

#### 3. Launch Development Environment

1. **Open project** in VS Code
2. **Command Palette** (`Ctrl+Shift+P`)
3. **Run**: "Remote-Containers: Reopen in Container"

### SSH Remote Development

For remote development on another machine:

#### 1. Configure SSH Connection

1. **Install Remote-SSH extension**
2. **Add SSH host** in VS Code:
   - Command Palette → "Remote-SSH: Add New SSH Host"
   - Enter: `ssh user@remote-host`

3. **Connect to remote host**:
   - Command Palette → "Remote-SSH: Connect to Host"

#### 2. Setup Remote Environment

Once connected to remote host:

1. **Clone your project** (if not already present)
2. **Start DNA container**:
   ```bash
   dna build develop
   dna up
   ```

3. **Open project folder** in VS Code

## Remote Development Workflows

### Local Development with Remote Execution

**Scenario**: Edit locally, execute in remote container

1. **Setup file synchronization**:
   ```bash
   # Using rsync
   rsync -avz --exclude='.git' ./ user@remote:/path/to/project/
   
   # Using VS Code Remote-SSH with auto-sync
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
   CONTAINER_SSH_PORT=2222
   
   # Developer 2
   CONTAINER_SSH_PORT=2223
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

#### VS Code Debugging

Create `.vscode/launch.json`:

```json
{
    "version": "0.2.0",
    "configurations": [
        {
            "name": "Python: ROS Node",
            "type": "python",
            "request": "launch",
            "program": "${workspaceFolder}/src/your_package/your_script.py",
            "console": "integratedTerminal",
            "env": {
                "ROS_DOMAIN_ID": "42",
                "PYTHONPATH": "/opt/ros/humble/lib/python3.10/site-packages:/ros2_ws/install/lib/python3.10/site-packages"
            },
            "cwd": "/ros2_ws"
        }
    ]
}
```

### C++ Debugging

#### Configure GDB for Container

1. **Install GDB** in container (usually pre-installed)
2. **Build with debug symbols**:
   ```bash
   colcon build --cmake-args -DCMAKE_BUILD_TYPE=Debug
   ```

3. **Configure IDE** to use remote GDB

#### VS Code C++ Debugging

Create `.vscode/launch.json` entry:

```json
{
    "name": "C++: ROS Node",
    "type": "cppdbg",
    "request": "launch",
    "program": "/ros2_ws/install/your_package/lib/your_package/your_node",
    "args": [],
    "stopAtEntry": false,
    "cwd": "/ros2_ws",
    "environment": [
        {"name": "ROS_DOMAIN_ID", "value": "42"}
    ],
    "MIMode": "gdb",
    "setupCommands": [
        {
            "description": "Enable pretty-printing for gdb",
            "text": "-enable-pretty-printing",
            "ignoreFailures": true
        }
    ]
}
```

## Testing Integration

### PyTest Integration

#### PyCharm Setup

1. **Configure test runner**:
   - `File > Settings > Tools > Python Integrated Tools`
   - **Default test runner**: pytest

2. **Set test paths**:
   - **Test source folders**: `tests/`
   - **Working directory**: Project root

#### VS Code Setup

Configure `.vscode/settings.json`:

```json
{
    "python.testing.pytestEnabled": true,
    "python.testing.pytestArgs": [
        "tests/"
    ],
    "python.testing.cwd": "/ros2_ws/src/${workspaceFolderBasename}"
}
```

### ROS Testing

#### Colcon Test Integration

Create test tasks in `.vscode/tasks.json`:

```json
{
    "version": "2.0.0",
    "tasks": [
        {
            "label": "colcon: test",
            "type": "shell",
            "command": "colcon",
            "args": ["test", "--packages-select", "${workspaceFolderBasename}"],
            "group": "test",
            "options": {
                "cwd": "/ros2_ws"
            }
        },
        {
            "label": "colcon: test-result",
            "type": "shell",
            "command": "colcon",
            "args": ["test-result", "--verbose"],
            "group": "test",
            "options": {
                "cwd": "/ros2_ws"
            }
        }
    ]
}
```

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

#### VS Code: "Failed to connect to remote extension host"

**Problem**: Remote extension host connection fails.

**Solutions**:
1. Restart VS Code
2. Clear remote connection cache
3. Check container logs: `docker logs CONTAINER_NAME`

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
   CONTAINER_SSH_PORT=2223
   CONTAINER_VNC_PORT=5902
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
