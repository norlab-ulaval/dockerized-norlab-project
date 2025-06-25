# dna config

Show Docker Compose configuration (In Development).

## Synopsis

```bash
dna config MODE [PLATFORM] [OPTIONS]
```

## Description

The `dna config` command displays the resolved Docker Compose configuration for different DNA modes and platforms. This is useful for debugging configuration issues, understanding service definitions, and validating environment variable interpolation.

> **⚠️ Note**: This command is currently in development and not yet fully released.

## Modes

| Mode | Description |
|------|-------------|
| `dev` | Development mode configuration |
| `deploy` | Deployment mode configuration |
| `ci-tests` | CI tests mode configuration |
| `slurm` | SLURM mode configuration |
| `release` | Release mode configuration |

## Platforms

| Platform | Description |
|----------|-------------|
| `darwin` | macOS configuration |
| `linux` | Linux configuration |
| `jetson` | NVIDIA Jetson configuration |

## Options

| Option | Description |
|--------|-------------|
| `--help`, `-h` | Show help message and exit |

## Examples

### Basic Configuration Display

```bash
# Show development configuration for current platform
dna config dev

# Show deployment configuration
dna config deploy

# Show CI tests configuration
dna config ci-tests
```

### Platform-Specific Configuration

```bash
# Show development configuration for macOS
dna config dev darwin

# Show development configuration for Linux
dna config dev linux

# Show development configuration for Jetson
dna config dev jetson
```

### Advanced Usage

```bash
# Show SLURM configuration
dna config slurm

# Show release configuration
dna config release
```

## What it Shows

The command displays the resolved Docker Compose configuration including:

### Service Definitions
- **Container configurations**: Image names, build contexts, environment variables
- **Volume mounts**: Host paths, container paths, mount options
- **Network settings**: Port mappings, network configurations
- **Resource limits**: CPU, memory, and GPU constraints

### Environment Variables
- **Resolved values**: Final values after variable interpolation
- **Source tracking**: Which files contribute to each variable
- **Platform overrides**: Platform-specific variable values

### Build Configurations
- **Build contexts**: Dockerfile locations and build arguments
- **Target stages**: Multi-stage build targets
- **Platform settings**: Architecture-specific build options

## Use Cases

### Configuration Debugging

```bash
# Debug development configuration issues
dna config dev

# Check if environment variables are resolved correctly
dna config dev | grep -A 5 environment

# Verify volume mount configurations
dna config dev | grep -A 10 volumes
```

### Platform Validation

```bash
# Validate configuration for different platforms
dna config dev darwin
dna config dev linux
dna config dev jetson

# Compare configurations across platforms
diff <(dna config dev darwin) <(dna config dev linux)
```

### CI/CD Integration

```bash
# Validate CI configuration in pipeline
dna config ci-tests

# Check SLURM configuration for cluster deployment
dna config slurm
```

### Documentation and Sharing

```bash
# Generate configuration documentation
dna config deploy > deployment-config.yaml

# Share configuration with team
dna config dev > team-dev-config.yaml
```

## Configuration Sources

The displayed configuration is assembled from multiple sources:

### Docker Compose Files
- **Base configuration**: Core service definitions
- **Platform overrides**: Platform-specific modifications
- **Mode-specific**: Development, deployment, CI, SLURM configurations

### Environment Files
- **Project variables**: `.env`, `.env.dna`, `.env.local`
- **DNA variables**: Internal DNA configuration
- **Platform variables**: Platform-specific overrides

### Build Contexts
- **Dockerfile**: Container build instructions
- **Build arguments**: Build-time variables
- **Multi-stage targets**: Specific build stages

## Output Format

The command outputs standard Docker Compose YAML format:

```yaml
services:
  develop:
    image: project/develop:latest
    build:
      context: .
      dockerfile: .dockerized_norlab/configuration/Dockerfile
      target: develop-stage
    environment:
      - ROS_DISTRO=humble
      - SUPER_PROJECT_NAME=my-project
    volumes:
      - ./src:/ros2_ws/src/my-project:rw
      - ./artifact:/artifact:rw
    ports:
      - "2222:22"
      - "7777:7777"
```

## Troubleshooting

### Command Not Available

**Problem**: "Command not released yet" message.

**Explanation**: The config command is still in development.

**Alternatives**:
1. **Use docker-compose directly**: 
   ```bash
   docker-compose -f .dockerized_norlab/configuration/docker-compose.yml config
   ```

2. **Use dna project dotenv**: Check environment variables
   ```bash
   dna project dotenv
   ```

### Configuration Errors

**Problem**: Invalid configuration displayed.

**Solutions**:
1. **Validate environment**: Check environment variable resolution
   ```bash
   dna project dotenv
   ```

2. **Check compose files**: Validate Docker Compose syntax
   ```bash
   docker-compose -f compose-file.yml config --quiet
   ```

### Platform Issues

**Problem**: Platform-specific configuration not working.

**Solutions**:
1. **Check platform detection**: Verify platform is correctly detected
2. **Use explicit platform**: Specify platform explicitly
   ```bash
   dna config dev linux  # instead of just 'dna config dev'
   ```

## Development Status

### Current Implementation
- ✅ Basic command structure
- ✅ Mode and platform parsing
- ✅ Docker Compose file selection
- ⚠️ Limited functionality (in development)

### Planned Features
- 🔄 Full mode support
- 🔄 Enhanced platform detection
- 🔄 Configuration validation
- 🔄 Output formatting options

## See Also

- [dna project](project.md) - Project configuration management
- [dna build](build.md) - Build container images
- [Project Initialization & Configuration](../project_initialization_and_configuration.md) - Configuration guide

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
