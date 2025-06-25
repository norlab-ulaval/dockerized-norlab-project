# dna exec

Execute commands in a running DNA container.

## Synopsis

```bash
dna exec [OPTIONS] [SERVICE] [--] COMMAND [ARGS...]
```

## Description

The `dna exec` command executes commands and arguments in a running DNA container. This is useful for running specific commands, scripts, or tools within the containerized environment without needing to attach to the container interactively.

## Services

| Service | Description |
|---------|-------------|
| `develop` | Execute in development service (default) |
| `deploy` | Execute in deployment service |

## Options

| Option | Description |
|--------|-------------|
| `-e`, `--env stringArray` | Set container environment variables |
| `-w`, `--workdir string` | Override path to working directory |
| `-T`, `--no-TTY` | Disable pseudo-TTY allocation |
| `--detach` | Execute COMMAND in the background |
| `--dry-run` | Dry run mode (requires --detach flag) |
| `--help`, `-h` | Show help message and exit |

## Positional Arguments

| Argument | Description |
|----------|-------------|
| `COMMAND [ARGS...]` | Command and arguments to execute inside the container (default: bash) |

## Examples

### Basic Command Execution

```bash
# Execute a simple command
dna exec ls -la

# Execute with specific service
dna exec develop python3 --version
dna exec deploy ./deploy_script.sh
```

### Working Directory Override

```bash
# Execute command in specific directory
dna exec --workdir "/" -- bash -c 'tree -L 1 -a $(pwd)'

# Run tests from test directory
dna exec --workdir "/ros2_ws" -- colcon test
```

### Environment Variables

```bash
# Set environment variables
dna exec -e "DEBUG=true" -e "LOG_LEVEL=info" -- python3 debug_script.py

# Multiple environment variables
dna exec --env "ROS_DOMAIN_ID=42" --env "VERBOSE=1" -- ros2 node list
```

### Background Execution

```bash
# Run command in background
dna exec --detach -- ros2 launch my_package my_launch.py

# Background with dry run
dna exec --dry-run --detach -- long_running_process.sh
```

### Advanced Examples

```bash
# Complex command with shell interpretation
dna exec -- bash -c 'echo "Hello from container: $(hostname)"'

# Execute without TTY (useful for scripts)
dna exec -T -- python3 automated_script.py

# Chain multiple commands
dna exec -- bash -c 'cd /ros2_ws && source install/setup.bash && ros2 run my_package my_node'
```

## Use Cases

### Development Tasks

```bash
# Build ROS workspace
dna exec --workdir "/ros2_ws" -- colcon build

# Run tests
dna exec -- pytest tests/

# Install Python packages
dna exec -- pip3 install numpy matplotlib
```

### System Administration

```bash
# Check system status
dna exec -- systemctl status ssh

# View logs
dna exec -- journalctl -u my_service --since "1 hour ago"

# Update packages
dna exec -- apt-get update && apt-get upgrade -y
```

### Debugging and Inspection

```bash
# Check environment variables
dna exec -- env | grep ROS

# Inspect file system
dna exec -- find /ros2_ws -name "*.py" | head -10

# Check running processes
dna exec -- ps aux
```

## Execution vs. Attachment

### When to use `dna exec`

- **Single commands**: For executing specific commands
- **Automation**: For scripted operations
- **Background tasks**: For non-interactive operations
- **CI/CD pipelines**: For automated testing and deployment

### When to use `dna attach` instead

- **Interactive development**: When you need a full shell session
- **Long-running sessions**: For extended development work
- **Container exploration**: When you want to explore the container environment

## Command Behavior

### Default Behavior

- **Working directory**: Uses container's default working directory
- **Environment**: Inherits container's environment variables
- **User context**: Runs as the configured container user
- **TTY allocation**: Automatically allocated for interactive commands

### Service Discovery

The `exec` command automatically discovers the appropriate service:

1. **Explicit service**: If you specify `develop` or `deploy`
2. **Offline deployment**: Automatically detects offline deployment services
3. **Default fallback**: Uses `develop` service if no other option

## Background Execution

### Detached Mode

```bash
# Start long-running process in background
dna exec --detach -- python3 long_running_script.py

# Check if process is still running
docker exec CONTAINER_NAME ps aux | grep python3
```

### Process Management

```bash
# Start background service
dna exec --detach -- ros2 launch my_package service.launch.py

# Monitor logs
docker logs -f CONTAINER_NAME

# Stop background process (if needed)
dna exec -- pkill -f "ros2 launch"
```

## Troubleshooting

### Command Not Found

**Problem**: "command not found" error.

**Solutions**:
1. **Check PATH**: Verify command is in container's PATH
   ```bash
   dna exec -- which python3
   dna exec -- echo $PATH
   ```

2. **Use full path**: Specify complete path to executable
   ```bash
   dna exec -- /usr/bin/python3 script.py
   ```

3. **Install missing packages**: Install required software
   ```bash
   dna exec -- apt-get update && apt-get install -y package-name
   ```

### Permission Denied

**Problem**: Permission errors when executing commands.

**Solutions**:
1. **Check file permissions**:
   ```bash
   dna exec -- ls -la /path/to/file
   ```

2. **Use appropriate user**: Run as root if necessary
   ```bash
   dna exec --user root -- command
   ```

3. **Fix ownership**: Correct file ownership
   ```bash
   dna exec -- sudo chown user:group /path/to/file
   ```

### Environment Issues

**Problem**: Environment variables not set correctly.

**Solutions**:
1. **Set explicitly**: Use `-e` flag
   ```bash
   dna exec -e "VAR=value" -- command
   ```

2. **Source environment**: Load environment files
   ```bash
   dna exec -- bash -c 'source /opt/ros/humble/setup.bash && command'
   ```

### Container Not Running

**Problem**: Cannot execute command because container is not running.

**Solutions**:
1. **Start container**: Ensure container is running
   ```bash
   dna up --no-attach
   ```

2. **Check container status**:
   ```bash
   docker ps
   ```

## Integration with CI/CD

### Automated Testing

```bash
# Run unit tests
dna exec --no-TTY -- pytest tests/ --junit-xml=results.xml

# Run integration tests
dna exec --detach deploy -- ./integration_tests.sh
```

### Build Automation

```bash
# Build project
dna exec --workdir "/ros2_ws" -- colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Package application
dna exec -- ./package_script.sh
```

## Performance Considerations

- **Command overhead**: Each `exec` call has minimal overhead
- **Resource sharing**: Commands share container resources
- **Network access**: Commands have same network access as container
- **File system**: Commands operate on container's file system

## See Also

- [dna attach](attach.md) - Attach to running containers
- [dna up](up.md) - Start containers
- [dna run](run.md) - Run commands in new containers
- [Project Initialization & Configuration](../project_initialization_and_configuration.md) - Container configuration

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
