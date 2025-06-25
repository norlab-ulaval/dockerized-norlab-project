# dna down

Stop DNA containers and clean up resources.

## Synopsis

```bash
dna down [OPTIONS] [<any-docker-compose-down-flags>]
```

## Description

The `dna down` command stops and removes DNA containers, networks, and associated resources. It provides a clean way to shut down your containerized development environment while preserving persistent data.

## Options

| Option | Description |
|--------|-------------|
| `--slurm` | Stop project-related SLURM containers |
| `--help`, `-h` | Show help message and exit |
| `<docker-compose-down-flags>` | Pass additional flags directly to docker-compose down |

## Examples

### Basic Container Shutdown

```bash
# Stop all DNA containers
dna down

# Stop containers and remove volumes (careful - this removes persistent data)
dna down --volumes

# Stop containers and remove images
dna down --rmi all
```

### SLURM Container Management

```bash
# Stop SLURM-related containers
dna down --slurm
```

### Advanced Cleanup

```bash
# Stop containers and remove orphaned containers
dna down --remove-orphans

# Force stop and remove everything
dna down --volumes --remove-orphans --rmi all
```

## What it does

When you run `dna down`, the command will:

1. **Stop running containers**: Gracefully stops all DNA containers
2. **Remove containers**: Removes stopped containers from the system
3. **Clean up networks**: Removes Docker networks created for the project
4. **Preserve volumes**: Keeps persistent data volumes by default (unless `--volumes` is specified)

## Container Lifecycle

### Normal Workflow
```bash
# Start containers
dna up

# Work in containers
# ... development work ...

# Stop containers when done
dna down
```

### Data Preservation

By default, `dna down` preserves:
- **Artifact data**: Contents of `artifact/` directory
- **External data**: Mounted external datasets
- **Container volumes**: Any named volumes defined in docker-compose

## Docker Compose Integration

The `down` command uses docker-compose configurations and supports all standard docker-compose down flags:

### Common Docker Compose Flags

| Flag | Description |
|------|-------------|
| `--volumes` | Remove named volumes and anonymous volumes |
| `--remove-orphans` | Remove containers for services not defined in compose file |
| `--rmi all` | Remove all images used by any service |
| `--rmi local` | Remove only images that don't have a custom tag |
| `--timeout TIMEOUT` | Specify shutdown timeout in seconds |

## Use Cases

### Development Workflow
```bash
# End development session
dna down

# Clean restart (removes containers but keeps data)
dna down && dna up
```

### System Maintenance
```bash
# Full cleanup (removes everything including data)
dna down --volumes --rmi all --remove-orphans
```

### Resource Management
```bash
# Stop containers to free system resources
dna down

# Later, restart without rebuilding
dna up
```

## Troubleshooting

### Containers Won't Stop

**Problem**: Containers don't stop gracefully.

**Solutions**:
1. **Force stop**: Use docker-compose timeout
   ```bash
   dna down --timeout 10
   ```

2. **Manual intervention**: Stop containers directly
   ```bash
   docker stop $(docker ps -q --filter "label=com.docker.compose.project")
   ```

### Permission Errors

**Problem**: Permission denied when removing containers.

**Solutions**:
1. **Check Docker permissions**: Ensure user is in docker group
2. **Use sudo**: If necessary for system-wide installations
   ```bash
   sudo dna down
   ```

### Volumes Not Removed

**Problem**: Data persists after `dna down`.

**Explanation**: This is normal behavior. Use `--volumes` flag to remove persistent data:
```bash
dna down --volumes
```

### Network Conflicts

**Problem**: Network removal fails due to active endpoints.

**Solutions**:
1. **Remove orphaned containers**:
   ```bash
   dna down --remove-orphans
   ```

2. **Manual network cleanup**:
   ```bash
   docker network prune
   ```

## Safety Considerations

### Data Loss Prevention

⚠️ **Warning**: Using `--volumes` will permanently delete:
- Artifact data
- Database contents
- Any persistent application data

Always backup important data before using destructive flags.

### Recommended Practices

1. **Regular cleanup**: Run `dna down` when not actively developing
2. **Selective removal**: Use specific flags rather than `--volumes` unless necessary
3. **Backup strategy**: Regularly backup `artifact/` directory contents

## See Also

- [dna up](up.md) - Start containers
- [dna build](build.md) - Build container images
- [dna attach](attach.md) - Attach to running containers
- [Project Initialization & Configuration](../project_initialization_and_configuration.md) - Container configuration

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
