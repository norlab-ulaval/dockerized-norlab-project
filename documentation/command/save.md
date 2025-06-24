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
| `SERVICE` | Service to save (`develop` or `deploy`) |

## Options

| Option | Description |
|--------|-------------|
| `--help`, `-h` | Show help message and exit |

## Services

| Service | Description | Contents |
|---------|-------------|----------|
| `develop` | Development service | Docker image only (assumes project is cloned on target) |
| `deploy` | Deployment service | Full project structure for self-contained deployment |

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

## See Also

- [dna load](load.md) - Load Docker images from files
- [dna build](build.md) - Build container images
- [Installation Guide](../install.md) - Offline installation methods

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
