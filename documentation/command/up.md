# dna up

Start and attach to DNA containers in daemon mode.

## Synopsis

```bash
dna up [OPTIONS] [SERVICE] [-- COMMAND [ARGS...]]
```

## Description

The `dna up` command starts DNA containers in daemon mode and optionally attaches to them or executes commands within them. This is equivalent to running `docker compose up --detach && docker compose attach`, ensuring containers continue running in the background even after you exit.

## Services

| Service | Description |
|---------|-------------|
| `develop` | Start development service (default) |
| `deploy` | Start deployment service |

## Options

### Basic Options

| Option | Description |
|--------|-------------|
| `--no-attach` | Don't attach to started container (not compatible with execute options) |
| `--help`, `-h` | Show help message and exit |

### Execute Options

| Option | Description |
|--------|-------------|
| `-e`, `--env stringArray` | Set container environment variables |
| `-w`, `--workdir string` | Override path to working directory |
| `-T`, `--no-TTY` | Disable pseudo-TTY allocation |
| `--detach` | Execute COMMAND in the background |
| `--dry-run` | Dry run mode (requires --detach flag) |

## Positional Arguments

| Argument | Description |
|----------|-------------|
| `COMMAND [ARGS...]` | Command and arguments to execute inside the container (default: bash) |

## Behavior

1. **Service Selection**: Automatically selects device and architecture-specific docker-compose configuration
2. **Daemon Mode**: Containers start in daemon mode and continue running in background
3. **Auto-attach**: By default, attaches to the started container unless `--no-attach` is specified
4. **Offline Detection**: Automatically detects and uses offline deployment services when available

## Examples

### Basic Container Startup

```bash
# Start develop service and attach
dna up

# Start deploy service and attach
dna up deploy
```

### Start Without Attaching

```bash
# Start container but don't attach to it
dna up --no-attach

# Start specific service without attaching
dna up --no-attach deploy
```

### Execute Commands

```bash
# Execute a specific command
dna up -- python3 my_script.py

# Execute command with custom working directory
dna up --workdir "/ros2_ws" -- colcon build

# Execute command in background
dna up --detach -- ros2 launch my_package my_launch.py
```

### Environment Variables

```bash
# Set environment variables
dna up -e "DEBUG=true" -e "LOG_LEVEL=info" -- python3 debug_script.py

# Multiple environment variables
dna up --env "ROS_DOMAIN_ID=42" --env "VERBOSE=1"
```

### Advanced Examples

```bash
# Execute complex command with custom workdir
dna up --workdir "/" -- bash -c 'tree -L 1 -a $(pwd)'

# Run tests in background
dna up --detach deploy -- pytest tests/

# Start with no TTY (useful for scripts)
dna up -T -- python3 automated_script.py
```

## Workflow Integration

### Development Workflow

```bash
# 1. Build development image
dna build develop

# 2. Start and attach to container
dna up

# 3. Work inside container
# (container continues running in background when you exit)

# 4. Re-attach later if needed
dna attach
```

### Deployment Workflow

```bash
# 1. Build deployment image
dna build deploy

# 2. Start deployment service
dna up deploy

# 3. Execute deployment commands
dna up deploy -- ./deploy_script.sh
```

### Testing Workflow

```bash
# Start container and run tests
dna up -- pytest tests/

# Run tests in background
dna up --detach -- pytest tests/ --junit-xml=results.xml
```

## Container Lifecycle

### Container States

1. **Not Started**: Container doesn't exist
2. **Starting**: Container is being created and started
3. **Running**: Container is running in daemon mode
4. **Attached**: You're connected to the running container
5. **Detached**: Container continues running in background

### State Transitions

```bash
# Start container (not started → running + attached)
dna up

# Detach from container (attached → detached, container still running)
# Press Ctrl+P, Ctrl+Q or exit

# Re-attach to running container (detached → attached)
dna attach

# Stop container (running → not started)
dna down
```

## Configuration

### Docker Compose Integration

The `up` command uses docker-compose configurations:

- **Base config**: `.dockerized_norlab/configuration/docker-compose.yml`
- **Device-specific**: Automatically selected based on platform
- **Architecture-specific**: Automatically selected based on CPU architecture

### Service Discovery

DNA automatically discovers available services:

1. **Explicit service**: If you specify `develop` or `deploy`
2. **Offline deployment**: Automatically detected for offline scenarios
3. **Default fallback**: Uses `develop` service if no other option

## Troubleshooting

### Container Won't Start

**Problem**: Container fails to start.

**Solutions**:
1. Check if images are built: `dna build develop`
2. Verify docker-compose configuration
3. Check for port conflicts
4. Review container logs: `docker logs DN_CONTAINER_NAME`

### Cannot Attach to Container

**Problem**: Attachment fails or connection is refused.

**Solutions**:
1. Verify container is running: `docker ps`
2. Check SSH service in container: `dna exec systemctl status ssh`
3. Verify port mappings: `docker port DN_CONTAINER_NAME`

### Permission Denied

**Problem**: Permission errors when executing commands.

**Solutions**:
1. Check user configuration in `.env`:
   ```bash
   SUPER_PROJECT_USER=$(id -un)
   ```
2. Verify file permissions in mounted volumes
3. Use appropriate user context: `dna exec --user root COMMAND`

### Port Conflicts

**Problem**: Container ports are already in use.

**Solutions**:
1. Change ports in `.env.local`:
   ```bash
   DN_SSH_SERVER_PORT=2223
   DN_GDB_SERVER_PORT=7777
   ```
2. Stop conflicting containers: `docker ps` and `docker stop`

### Environment Variables Not Set

**Problem**: Custom environment variables not available in container.

**Solutions**:
1. Use `-e` flag: `dna up -e "VAR=value"`
2. Add to `.env` file and rebuild container
3. Check environment variable precedence order

## Performance Tips

- **Use `--no-attach`** when you only need to start services
- **Use `--detach`** for long-running background processes
- **Reuse running containers** instead of stopping/starting frequently
- **Use specific services** (`develop` vs `deploy`) for optimal resource usage

## See Also

- [dna build](build.md) - Build container images
- [dna attach](attach.md) - Attach to running containers
- [dna down](down.md) - Stop containers
- [dna exec](exec.md) - Execute commands in containers
- [Project Initialization & Configuration](../project_initialization_and_configuration.md) - Container configuration

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
