# dna build

Build project Docker images for different deployment scenarios.

## Synopsis

```bash
dna build [OPTIONS] [SERVICE] [-- <any-docker-argument>]
```

## Description

The `dna build` command builds Docker images for your DNA project. It supports different build modes and services depending on your use case, from development to deployment and testing scenarios.

## Services

| Service    | Description                                                   |
|------------|---------------------------------------------------------------|
| `develop`  | Build development images for interactive development          |
| `deploy`   | Build deployment images for production environments           |
| `ci-tests` | Build images optimized for continuous integration testing     |
| `slurm`    | Build images for SLURM cluster job execution                  |
| `core`        | Build the core image, the base image for all other DNA images |
| `release`  | Build release images (🚧 In development)                      |

## Options

| Option | Description                                                                                                          |
|--------|----------------------------------------------------------------------------------------------------------------------|
| `--multiarch` | Build services for multiple architectures                                      |
| `--rmab` | Create/re-create a docker buildx multiarch builder instance                                                          |
| `--online-build` | Build images sequentially by pushing/pulling intermediate images from Docker Hub (requires Docker Hub authentication) |
| `--save DIRPATH` | Save built image to specified directory (develop or deploy services only)                                            |
| `--push` | Push image to Docker Hub (deploy services only, requires Docker Hub authentication). For slurm with `--apptainer`, selects the registry push pipeline (see `--apptainer`). |
| `--apptainer <profile>` | HPC Apptainer workflow for slurm service only. **Requires `--save` or `--push`:** `--save` selects the tar archive pipeline (saves `.tar`, generates `dna_tar_to_apptainer_sif_converter.sh`); `--push` selects the registry pipeline (pushes to Docker registry, generates `dna_registry_to_apptainer_sif_converter.sh`). Does **not** execute apptainer locally (macOS compatible). See [Apptainer documentation](apptainer.md). |
| `--save` | When used with `--apptainer <profile>`, selects the tar archive pipeline (no `DIRPATH` needed). When used for `develop`/`deploy` services (without `--apptainer`), saves the built image to the specified `DIRPATH`. |
| `--squash` | Squash the built image to reduce its size. For `slurm` (with or without `--apptainer`): squashes before saving tar archive, pushing to registry, or in-place. For `deploy`/`ci-tests`: squashes image in-place after building. Uses `docker export/import` — **loses image history and metadata**. |
| `--help`, `-h` | Show help message and exit                                                                                           |
| `-- <docker-args>` | Pass additional arguments directly to Docker build                                                                   |

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
# Build for multiple architectures
dna build --multiarch develop

# Build for multiple architectures with builder recreation
dna build --multiarch --rmab develop
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

# Build slurm image and save as tar archive for HPC Apptainer workflow (Valeria) — tar archive pipeline
dna build slurm --apptainer valeria --save

# Build slurm image and push to Docker registry for HPC Apptainer workflow — registry push pipeline
dna build slurm --apptainer valeria --push

# Build and squash slurm image before saving (reduces transfer size)
dna build slurm --apptainer valeria --save --squash

# Build, push to registry, and squash before push
dna build slurm --apptainer valeria --push --squash

# Build slurm image and squash in-place (no Apptainer artifacts)
dna build slurm --squash
```

### Squash Image to Reduce Size

```bash
# Squash deploy image in-place after build
dna build deploy --squash

# Squash ci-tests image in-place after build
dna build ci-tests --squash
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
**Solution 1**: Execute `dna build` with `--multiarch --rmab` to re-create the docker buildx multiarch builder `local-builder-multiarch-virtual`. 

**Solution 2**: Instanciate via script a Docker Buildx builder with multi-architecture support by executing
```bash
bash src/lib/core/utils/buildx_builder.bash
```

**Solution 3**: Instanciate manualy a Docker Buildx builder with multi-architecture support:
```bash
docker buildx create --name local-builder-multiarch-virtual --driver=docker-container --platform linux/amd64,linux/arm64 --bootstrap
docker buildx ls
```
**Note**: `local-builder-multiarch-virtual` is the default multi-architecture builder name use by `dna`. To use a diferent one, just set `BUILDX_BUILDER` environment variable in the same shell you are executing `dna` commands:
```bash
export BUILDX_BUILDER=my-multiarch-builder 
dna build --multiarch
```

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
- [Apptainer / HPC Workflow](apptainer.md) - Complete guide for HPC/Apptainer deployment
- [Docker Buildx documentation](https://docs.docker.com/buildx/)

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
