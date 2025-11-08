# dna config

Show Docker Compose configuration file.

## Synopsis

```bash
dna config [OPTIONS] MODE [--] [DOCKER_CONFIG_FLAGS|DOCKER_BAKE_FLAGS]
```

## Description

The `dna config` command displays the resolved Docker Compose configuration for different DNA modes and platforms. This is useful for debugging configuration issues, understanding service definitions, and validating environment variable interpolation.

The command uses `docker compose config` or `docker buildx bake` under the hood and can consume their respective option flags.

### Features
- **Multiple output formats**: Docker Compose YAML, Docker Buildx Bake JSON
- **Extensible**: Supports additional Docker command flags via pass-through
- **Docker integration**: Native docker compose config and docker buildx bake support
- **Flexible mode support**: Development, deployment, CI/CD, SLURM, and build configurations
- **Architecture support**: Native and multi-architecture
- **Platform support**: MacOs, Ubunt, L4T (Jetson OS)


## Options

| Option | Description |
|--------|-------------|
| `--bake` | Use 'docker buildx bake' instead of 'docker compose config' |
| `--compose-to-bake` | Print the compose file converted to bake format |
| `-q`, `--quiet` | Skip DNA messages, only print docker command output |
| `--help`, `-h` | Show help message and exit |

## Modes

| Mode | Description |
|------|-------------|
| `build-core` | Core only (pre, user, final) native build config |
| `build-core-ma` | Core only (pre, user, final) multi-architecture build config |
| `build` | All native build config |
| `build-ma` | All multi-architecture build config |
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

## Examples

### Basic Configuration Display

```bash
# Show development configuration for current platform
dna config dev

# Show deployment configuration
dna config deploy

# Show CI tests configuration
dna config ci-tests

# Show SLURM configuration
dna config slurm
```

### Build Configuration

```bash
# Show core-only native build configuration
dna config build-core

# Show core-only multi-architecture build configuration
dna config build-core-ma

# Show all native build configuration
dna config build

# Show all multi-architecture build configuration
dna config build-ma
```

### Platform-Specific Configuration

```bash
# Show development configuration for macOS
dna config dev darwin

# Show development configuration for Linux
dna config dev linux

# Show development configuration for Jetson
dna config dev jetson

# Show deployment configuration for specific platform
dna config deploy jetson
```

### Using Docker Buildx Bake

```bash
# Use docker buildx bake instead of docker compose config
dna config --bake build

# Convert compose file to bake format
dna config --compose-to-bake build-core

# Use bake with multi-architecture build
dna config --bake build-ma
```

### Quiet Mode and Docker Flags

```bash
# Skip DNA messages, show only docker output
dna config --quiet dev

# Pass additional docker compose config flags
dna config dev -- --services

# Pass docker compose config flags with quiet mode
dna config --quiet dev -- --volumes

# Use bake with additional docker buildx bake flags
dna config --bake build -- --load
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

# Debug with quiet mode to focus on docker output
dna config --quiet dev

# Check if environment variables are resolved correctly
dna config dev | grep -A 5 environment

# Verify volume mount configurations
dna config dev | grep -A 10 volumes

# Debug specific services only
dna config dev -- --services
```

### Build Configuration Analysis

```bash
# Analyze core build configuration
dna config build-core

# Compare native vs multi-architecture builds
dna config build > native-build.yaml
dna config build-ma > multiarch-build.yaml
diff native-build.yaml multiarch-build.yaml

# Use bake format for build analysis
dna config --bake build-core

# Convert compose to bake format
dna config --compose-to-bake build
```

### Platform Validation

```bash
# Validate configuration for different platforms
dna config dev darwin
dna config dev linux
dna config dev jetson

# Compare configurations across platforms
diff <(dna config dev darwin) <(dna config dev linux)

# Quiet comparison without DNA messages
diff <(dna config --quiet dev darwin) <(dna config --quiet dev linux)
```

### CI/CD Integration

```bash
# Validate CI configuration in pipeline
dna config ci-tests

# Check SLURM configuration for cluster deployment
dna config slurm

# Validate build configuration in CI
dna config --quiet build-ma -- --services

# Generate bake configuration for CI
dna config --bake build-ma > ci-bake-config.json
```

### Documentation and Sharing

```bash
# Generate configuration documentation
dna config deploy > deployment-config.yaml

# Share configuration with team
dna config dev > team-dev-config.yaml

# Generate build documentation
dna config build-core > build-config.yaml

# Create bake configuration files
dna config --bake build > build.json
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
      dockerfile: .dockerized_norlab/configuration/Dockerfile.project-core-user
      target: develop-stage
    environment:
      - ROS_DISTRO=humble
      - SUPER_PROJECT_NAME=my-project
    volumes:
      - ./src:/ros2_ws/src/my-project:rw
      - ./artifact:/artifact:rw
    ports:
      - "2222:2222"
      - "7777:7777"
```

## Troubleshooting

### Configuration Errors

**Problem**: Invalid configuration displayed or command fails.

**Solutions**:
1. **Use quiet mode**: Focus on docker output without DNA messages
   ```bash
   dna config --quiet dev
   ```

2. **Validate environment**: Check environment variable resolution
   ```bash
   dna project dotenv
   ```

3. **Check compose files**: Validate Docker Compose syntax directly
   ```bash
   docker compose -f .dockerized_norlab/core/docker/compose-file.yaml config --quiet
   ```

### Platform Issues

**Problem**: Platform-specific configuration not working.

**Solutions**:
1. **Use explicit platform**: Specify platform explicitly
   ```bash
   dna config dev linux  # instead of just 'dna config dev'
   ```

2. **Check supported platforms**: Verify the platform is supported (darwin, linux, jetson)

### Build Mode Issues

**Problem**: Build modes not working or showing unexpected results.

**Solutions**:
1. **Use appropriate build mode**: Choose the right mode for your needs
   ```bash
   dna config build-core    # For core-only builds
   dna config build         # For complete builds
   ```

2. **Try bake format**: Use bake format for build configuration analysis
   ```bash
   dna config --bake build-core
   ```

### Docker Flags Not Working

**Problem**: Additional Docker flags are not being passed correctly.

**Solutions**:
1. **Use double dash separator**: Separate DNA options from Docker flags
   ```bash
   dna config dev -- --services --volumes
   ```

2. **Check flag compatibility**: Ensure flags are compatible with the underlying Docker command
   - For compose config: `docker compose config --help`
   - For buildx bake: `docker buildx bake --help`


## See Also

- [dna project](project.md) - Project configuration management
- [dna build](build.md) - Build container images
- [Project Initialization & Configuration](../project_initialization_and_configuration.md) - Configuration guide

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
