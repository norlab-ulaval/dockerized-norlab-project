# dna save

Save Docker images to files for offline use.

## Synopsis

```bash
dna save [OPTIONS] DIRPATH SERVICE
```

## Description

The `dna save` command creates portable archives containing Docker images and necessary files for offline deployment. This enables transferring DNA projects to environments without internet access or for backup purposes.

## Arguments

| Argument | Description |
|----------|-------------|
| `DIRPATH` | Directory path where to save the image archive |
| `SERVICE` | Service to save (`develop`, `deploy`, or `slurm`) |

## Options

| Option | Description |
|--------|-------------|
| `--help`, `-h` | Show help message and exit |
| `--apptainer <profile>` | Generate Apptainer artifacts for HPC alongside the tar archive (slurm service only). Creates `dna_tar_to_apptainer_sif_converter.sh` helper script. `<profile>` selects `.env.<profile>` configuration (e.g., `valeria`, `compute_canada`). **Does not execute `apptainer` locally** (macOS compatible). |
| `--squash` | Squash the image before saving to reduce the archive size. Works for all supported services (`slurm`, `develop`, `deploy`). Uses `docker export/import` — **loses image history and metadata**. |

## Services

| Service | Description | Contents |
|---------|-------------|----------|
| `develop` | Development service | Docker image only (assumes project is cloned on target) |
| `deploy` | Deployment service | Full project structure for self-contained deployment |
| `slurm` | HPC / Apptainer service | Docker tar archive; add `--apptainer` to also generate `dna_tar_to_apptainer_sif_converter.sh` for HPC conversion |

## Output Structure

The command creates a directory with the following pattern:
```
dna-save-<SERVICE>-<REPO_NAME>-<timestamp>/
```

### Develop Service Output
```
dna-save-develop-my-project-202312151430/
├── my-project-develop.latest.tar    ← Docker image archive
├── README.md                        ← Usage instructions
└── load_instructions.txt            ← Loading instructions
```

### Deploy Service Output
```
dna-save-deploy-my-project-202312151430/
├── my-project-deploy.latest.tar     ← Docker image archive
├── project/                         ← Complete project structure
│   ├── .dockerized_norlab/          ← DNA configuration
│   ├── src/                         ← Source code
│   ├── tests/                       ← Test files
│   └── ...                          ← Other project files
├── README.md                        ← Usage instructions
└── deployment_instructions.txt      ← Deployment guide
```

## Examples

### Save Slurm Image (for HPC)

```bash
# Save slurm image as tar archive
dna save /output/dir slurm

# Save slurm image with Apptainer artifacts (generates dna_tar_to_apptainer_sif_converter.sh)
dna save --apptainer valeria /output/dir slurm

# Save slurm image squashed (reduces transfer size)
dna save --squash /output/dir slurm

# Save slurm image with Apptainer artifacts and squash
dna save --apptainer valeria --squash /output/dir slurm
```

### Save Development Image

```bash
# Save development image to current directory
dna save . develop

# Save to specific directory
dna save /backup/images develop

# Save to external drive
dna save /media/usb/dna-backups develop
```

### Save Deployment Package

```bash
# Save complete deployment package
dna save /deployment/packages deploy

# Save deployment package with squashed image (smaller archive)
dna save --squash /deployment/packages deploy

# Save to shared network location
dna save /shared/deployments deploy
```

### Typical Workflow

```bash
# 1. Build the image you want to save
dna build develop

# 2. Save the image
dna save ./backups develop

# 3. Transfer the created directory to target machine
# 4. On target machine, use dna load to restore
```

## Use Cases

### Offline Deployment

```bash
# On connected machine
dna build deploy
dna save /transfer deploy

# Transfer files to offline machine
# On offline machine
dna load /transfer/dna-save-deploy-project-*/
```

### Backup and Archival

```bash
# Create backup of current development state
dna save /backups develop

# Archive specific version for later use
git tag v1.0.0
dna build deploy
dna save /archives deploy
```

### CI/CD Artifact Storage

```bash
# In CI pipeline - save built images as artifacts
dna build deploy
dna save ${CI_ARTIFACTS_DIR} deploy

# Later stages can load and deploy
dna load ${CI_ARTIFACTS_DIR}/dna-save-deploy-*/
```

### Air-Gapped Environments

```bash
# Prepare deployment package on internet-connected machine
dna build deploy
dna save /secure-transfer deploy

# Transfer via secure media to air-gapped environment
# Deploy without internet access
```

## File Transfer Methods

### Local Transfer

```bash
# Copy to external drive
dna save /media/usb/dna-images develop
cp -r /media/usb/dna-images/dna-save-* /target/location/
```

### Network Transfer

```bash
# Save and transfer via SCP
dna save /tmp develop
scp -r /tmp/dna-save-* user@target-host:/deployment/
```

### Archive for Distribution

```bash
# Create compressed archive
dna save /tmp deploy
cd /tmp
tar -czf my-project-deploy.tar.gz dna-save-deploy-*
```

## Deployment Differences

### Develop Service
- **Assumption**: Target machine has the project repository
- **Contents**: Only Docker image
- **Use case**: Development environment setup
- **Target**: Developers with existing project checkout

### Deploy Service  
- **Assumption**: Target machine may not have project files
- **Contents**: Complete project structure + Docker image
- **Use case**: Production deployment
- **Target**: Production servers or clean environments

## Storage Requirements

### Image Sizes
- **Base images**: 2-4 GB typical
- **With dependencies**: 4-8 GB typical
- **Full development**: 6-12 GB typical

### Archive Sizes
- **Develop service**: Image size only
- **Deploy service**: Image size + project files
- **Compression**: TAR archives are uncompressed

## Troubleshooting

### Directory Not Found

**Problem**: "Directory does not exist" error.

**Solutions**:
1. **Create directory**: Ensure target directory exists
   ```bash
   mkdir -p /path/to/save/directory
   dna save /path/to/save/directory develop
   ```

2. **Check permissions**: Verify write access
   ```bash
   ls -ld /path/to/save/directory
   ```

### Insufficient Disk Space

**Problem**: Save operation fails due to disk space.

**Solutions**:
1. **Check available space**: Verify sufficient disk space
   ```bash
   df -h /path/to/save/directory
   ```

2. **Clean up space**: Remove unnecessary files
   ```bash
   docker system prune -a  # Clean Docker cache
   ```

3. **Use different location**: Save to location with more space
   ```bash
   dna save /larger/disk/path develop
   ```

### Image Not Found

**Problem**: Docker image not found for saving.

**Solutions**:
1. **Build image first**: Ensure image exists
   ```bash
   dna build develop  # or deploy
   dna save . develop
   ```

2. **Check image exists**: Verify image is available
   ```bash
   docker images | grep project-name
   ```

### Permission Denied

**Problem**: Cannot write to target directory.

**Solutions**:
1. **Check permissions**: Verify write access
   ```bash
   ls -ld /target/directory
   ```

2. **Use accessible location**: Save to user-writable directory
   ```bash
   dna save ~/dna-saves develop
   ```

## Performance Tips

- **Use local storage**: Avoid network drives for better performance
- **Clean up regularly**: Remove old save directories to save space
- **Compress for transfer**: Use tar/gzip for network transfer
- **Parallel operations**: Save multiple services simultaneously if needed

## Apptainer / HPC Workflow

For HPC servers using Apptainer (Valeria, Compute Canada, Mamba), use `--apptainer <profile>` with `SERVICE=slurm`:

```bash
# Save slurm image with Apptainer artifacts
dna save --apptainer valeria /output/dir slurm

# Squash before saving (reduces HPC transfer size)
dna save --apptainer valeria --squash /output/dir slurm
```

This generates:
- `<project>-slurm.<tag>.tar` — Docker tar archive compatible with Apptainer's `docker-archive:` bootstrap
- `dna_tar_to_apptainer_sif_converter.sh` — Helper script to run on HPC: converts tar → SIF, then **deletes the tar archive** to free disk space
- `meta.txt` — Includes `APPTAINER_PROFILE`, `APPTAINER_TARGET_PLATFORM` (from HPC profile, default: `linux/amd64`), `SIF_BUILD_CMD`

> ℹ️ Platform is enforced from the HPC profile's `APPTAINER_TARGET_PLATFORM` variable via
> `DOCKER_DEFAULT_PLATFORM`, ensuring cross-architecture builds on Apple Silicon Macs.

The saved tar archive is one part of the Apptainer pipeline. After saving, use `dna run slurm --ga`
to generate run scripts, or use the slurm job templates (`slurm_job.apptainer.<profile>.template.bash`)
for production job submission. See [Apptainer / HPC Workflow](apptainer.md) for the complete guide
and the [three pipeline artifacts](apptainer.md#understanding-the-apptainer-pipeline-artifacts).

## See Also

- [dna load](load.md) - Load Docker images from files
- [dna build](build.md) - Build container images
- [Installation Guide](../install.md) - Offline installation methods

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
