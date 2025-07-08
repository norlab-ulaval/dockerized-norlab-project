# dna build

Build project Docker images for different deployment scenarios.

## Synopsis

```bash
dna build [OPTIONS] [SERVICE] [-- <any-docker-argument>]
```

## Description

The `dna build` command builds Docker images for your DNA project. It supports different build modes and services depending on your use case, from development to deployment and testing scenarios.

## Services

| Service | Description |
|---------|-------------|
| `develop` | Build development images for interactive development |
| `deploy` | Build deployment images for production environments |
| `ci-tests` | Build images optimized for continuous integration testing |
| `slurm` | Build images for SLURM cluster job execution |
| `release` | Build release images (🚧 In development) |

## Options

| Option | Description |
|--------|-------------|
| `--multiarch` | Build services for multiple architectures (requires configured docker buildx multiarch builder) |
| `--online-build` | Build images sequentially by pushing/pulling intermediate images from Docker Hub (requires Docker Hub authentication) |
| `--save DIRPATH` | Save built image to specified directory (develop or deploy services only) |
| `--push` | Push image to Docker Hub (deploy services only, requires Docker Hub authentication) |
| `--help`, `-h` | Show help message and exit |
| `-- <docker-args>` | Pass additional arguments directly to Docker build |

## Default Behavior

- **Architecture**: Builds for host native architecture by default
- **Build mode**: Builds offline from the local image store by default
- **Internet**: Requires internet connection for building

## Examples

### Basic Development Build

```bash
# Build development images
dna build develop
```

### Multi-Architecture Build

```bash
# Build for multiple architectures (requires buildx setup)
dna build --multiarch develop
```

### Build and Save Images

```bash
# Build and save images to a directory
dna build --save ./saved-images develop
```

### Deploy Build with Push

```bash
# Build deploy images and push to Docker Hub
dna build --push deploy
```

### Online Build Mode

```bash
# Build using online mode (push/pull intermediate images)
dna build --online-build develop
```

### CI Testing Build

```bash
# Build images for continuous integration
dna build ci-tests
```

### SLURM Job Build

```bash
# Build images for SLURM cluster execution
dna build slurm
```

### Pass Docker Arguments

```bash
# Pass additional Docker build arguments
dna build develop -- --no-cache --progress=plain
```

## Service Details

### develop
- **Purpose**: Interactive development and debugging
- **Features**: Includes development tools, debuggers, and full source code access
- **Mount behavior**: Source code is mounted as volumes for live editing
- **Use case**: Local development, remote development, debugging

### deploy
- **Purpose**: Production deployment
- **Features**: Optimized for size and security, minimal tooling
- **Mount behavior**: Source code is copied during build (not mounted)
- **Use case**: Production servers, embedded systems, deployment

### ci-tests
- **Purpose**: Continuous integration testing
- **Features**: Includes testing frameworks and CI-specific tools
- **Mount behavior**: Source code copied for isolated testing
- **Use case**: Automated testing, CI/CD pipelines

### slurm
- **Purpose**: High-performance computing on SLURM clusters
- **Features**: Optimized for batch job execution
- **Mount behavior**: Configured for cluster storage systems
- **Use case**: Compute-intensive tasks, batch processing

## Requirements

- **Internet connection**: Required for building (downloads base images and dependencies)
- **Docker**: Docker Engine with BuildKit support
- **Multi-arch builds**: Requires `docker buildx` with configured multi-architecture builder
- **Docker Hub authentication**: Required for `--online-build` and `--push` operations
  - Run `docker login` to authenticate with Docker Hub before using these features
  - Authentication is automatically verified before build operations begin
  - Commands will fail with clear error messages if not authenticated

## Build Process

1. **Environment validation**: Checks for required tools and connectivity
2. **Configuration loading**: Loads project-specific build settings
3. **Base image preparation**: Downloads or updates base images
4. **Layer building**: Builds Docker layers according to service type
5. **Optimization**: Applies service-specific optimizations
6. **Output**: Creates tagged images ready for use

## Troubleshooting

### "Be advised, you are currently offline"
**Problem**: No internet connection detected.  
**Solution**: Ensure you have an active internet connection and try again.

### Multi-architecture build fails
**Problem**: `--multiarch` flag fails.  
**Solution**: Set up Docker Buildx with multi-architecture support:
```bash
docker buildx create --name local-builder-multiarch-virtual --driver=docker-container --driver-opt="default-load=true" --platform linux/amd64,linux/arm64 --bootstrap --buildkitd-flags '--allow-insecure-entitlement network.host'
docker buildx ls
```
**Note**: `local-builder-multiarch-virtual` is the default multi-architecture builder name use by `dna`

### Push fails with authentication error
**Problem**: Cannot push to Docker Hub.  
**Solution**: Authenticate with Docker Hub:
```bash
docker login
```

### Build fails with "no space left on device"
**Problem**: Insufficient disk space.  
**Solution**: Clean up Docker images and containers:
```bash
docker system prune -a
```

## Performance Tips

- **Use `--online-build`** for better caching when building on multiple machines
- **Use `--multiarch`** only when you need to support multiple architectures
- **Save images** with `--save` for offline deployment or backup
- **Clean up regularly** to avoid disk space issues

## See Also

- [dna up](up.md) - Start built containers
- [dna save](save.md) - Save images for offline use
- [dna load](load.md) - Load saved images
- [Docker Buildx documentation](https://docs.docker.com/buildx/)

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
