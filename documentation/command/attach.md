# dna attach

Attach to a running DNA container.

## Synopsis

```bash
dna attach [OPTIONS] [SERVICE]
```

## Description

The `dna attach` command connects you to a running DNA container, providing an interactive terminal session. This is useful when you want to reconnect to a container that's already running in the background.

## Services

| Service | Description |
|---------|-------------|
| `develop` | Attach to development service (default) |
| `deploy` | Attach to deployment service |

## Options

| Option | Description |
|--------|-------------|
| `--help`, `-h` | Show help message and exit |

## Examples

### Basic Attachment

```bash
# Attach to default develop service
dna attach

# Attach to specific service
dna attach develop
dna attach deploy
```

### Typical Workflow

```bash
# 1. Start container in background
dna up --no-attach

# 2. Later, attach to the running container
dna attach

# 3. Work in the container
# ... development work ...

# 4. Detach (Ctrl+P, Ctrl+Q) - container keeps running
# 5. Re-attach when needed
dna attach
```

## Behavior

### Service Discovery

The `attach` command automatically discovers the appropriate service:

1. **Explicit service**: If you specify `develop` or `deploy`
2. **Offline deployment**: Automatically detects offline deployment services
3. **Default fallback**: Uses `develop` service if no other option

### Container Requirements

- **Container must be running**: The target container must already be started
- **SSH access**: Container must have SSH service running for attachment
- **Proper configuration**: Container must be properly configured with DNA

## Attachment vs. Execution

### When to use `dna attach`

- **Interactive development**: When you need a full shell session
- **Long-running sessions**: For extended development work
- **Container exploration**: When you want to explore the container environment
- **Reconnection**: When reconnecting to a previously started container

### When to use `dna exec` instead

- **Single commands**: For executing specific commands
- **Automation**: For scripted operations
- **Background tasks**: For non-interactive operations

## Container States and Attachment

### Container Lifecycle

```bash
# Container not running
dna up                    # → Container running + attached

# Detach from container (Ctrl+P, Ctrl+Q)
                         # → Container running + detached

dna attach               # → Container running + attached

dna down                 # → Container stopped
```

### Detaching from Containers

To detach from a container without stopping it:

1. **Keyboard shortcut**: Press `Ctrl+P` followed by `Ctrl+Q`
2. **Exit command**: Type `exit` (this will stop the container)

## SSH Integration

The `attach` command uses SSH to connect to containers:

### Default SSH Configuration

- **Port**: Configured via `CONTAINER_SSH_PORT` (default: 2222)
- **User**: Configured via `SUPER_PROJECT_USER`
- **Host**: localhost (for local containers)

### SSH Key Management

DNA handles SSH key management automatically:
- Keys are generated during container startup
- Authentication is handled transparently
- No manual SSH configuration required

## Troubleshooting

### Cannot Attach to Container

**Problem**: Attachment fails with connection refused.

**Solutions**:
1. **Check container status**:
   ```bash
   docker ps
   ```

2. **Verify SSH service**:
   ```bash
   dna exec systemctl status ssh
   ```

3. **Check port mapping**:
   ```bash
   docker port CONTAINER_NAME
   ```

4. **Restart SSH service**:
   ```bash
   dna exec sudo systemctl restart ssh
   ```

### Container Not Running

**Problem**: "Container not found" or similar error.

**Solutions**:
1. **Start the container**:
   ```bash
   dna up
   ```

2. **Check available containers**:
   ```bash
   docker ps -a
   ```

### Permission Denied

**Problem**: SSH authentication fails.

**Solutions**:
1. **Check user configuration**:
   ```bash
   # Verify SUPER_PROJECT_USER in .env
   grep SUPER_PROJECT_USER .dockerized_norlab/configuration/.env
   ```

2. **Regenerate SSH keys**:
   ```bash
   dna down && dna up
   ```

### Multiple Services Running

**Problem**: Unclear which service to attach to.

**Solutions**:
1. **List running containers**:
   ```bash
   docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
   ```

2. **Specify service explicitly**:
   ```bash
   dna attach develop
   # or
   dna attach deploy
   ```

## Advanced Usage

### Remote Development

For remote development workflows:

```bash
# On remote machine
dna up --no-attach

# From local machine (if SSH is configured)
ssh -t user@remote-host "dna attach"
```

### Multiple Terminal Sessions

You can have multiple terminal sessions in the same container:

```bash
# Terminal 1
dna attach

# Terminal 2 (new session)
dna exec bash

# Terminal 3 (another new session)
dna exec bash
```

## Performance Considerations

- **Resource usage**: Attachment uses minimal additional resources
- **Network latency**: For remote containers, network latency affects responsiveness
- **Container resources**: Multiple sessions share container CPU and memory

## See Also

- [dna up](up.md) - Start containers
- [dna exec](exec.md) - Execute commands in containers
- [dna down](down.md) - Stop containers
- [IDE Integration](../ide_integration.md) - IDE setup for container development

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
