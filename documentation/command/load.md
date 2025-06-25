# dna load

Load Docker images from files for offline use.

## Synopsis

```bash
dna load [OPTIONS] [SAVE_DIR_PATH]
```

## Description

The `dna load` command loads Docker images from archives created by `dna save`. This enables deploying DNA projects in offline environments or restoring from backups. The command validates archive integrity and automatically configures the environment based on the loaded service type.

## Arguments

| Argument | Description |
|----------|-------------|
| `SAVE_DIR_PATH` | Path to the saved directory (dna-save-<SERVICE>-<REPO_NAME>-<timestamp>) |

## Options

| Option | Description |
|--------|-------------|
| `--help`, `-h` | Show help message and exit |

## Automatic Path Detection

If `SAVE_DIR_PATH` is not provided, the command automatically searches for DNA save directories:

1. **Current directory**: Checks if current directory contains `meta.txt`
2. **Super project root**: Searches for DNA configuration in parent directories
3. **Error**: Reports error if no valid save directory is found

## Examples

### Basic Loading

```bash
# Load from specific directory
dna load /path/to/dna-save-deploy-project-202312151430

# Load from current directory (if it's a save directory)
cd dna-save-develop-project-202312151430
dna load

# Load with automatic detection
dna load
```

### Typical Workflow

```bash
# 1. Transfer save directory to target machine
scp -r dna-save-deploy-project-* user@target:/deployment/

# 2. On target machine, load the archive
cd /deployment/dna-save-deploy-project-*
dna load

# 3. Start using the loaded project
dna up
```

## Service-Specific Behavior

### Develop Service Loading

```bash
# Load development image
dna load dna-save-develop-project-202312151430

# After loading:
# - Docker image is loaded into local registry
# - Executes alias dna-<PROJECT>-cd for navigation
# - Assumes project source code is already present
```

### Deploy Service Loading

```bash
# Load deployment package
dna load dna-save-deploy-project-202312151430

# After loading:
# - Docker image is loaded into local registry
# - Changes to the loaded project directory
# - Complete project structure is available
# - Ready for immediate deployment
```

## Use Cases

### Offline Deployment

```bash
# On target machine without internet
dna load /transfer/dna-save-deploy-project-*
dna up deploy
```

### Backup Restoration

```bash
# Restore from backup
dna load /backups/dna-save-develop-project-*
dna up
```

### CI/CD Artifact Loading

```bash
# In deployment stage
dna load ${CI_ARTIFACTS_DIR}/dna-save-deploy-*
dna run deploy -- ./deployment_script.sh
```

### Development Environment Setup

```bash
# Set up development environment from saved image
git clone https://github.com/user/project.git
cd project
dna load /shared/dna-save-develop-project-*
dna up
```

## Archive Validation

The load command performs several validation checks:

### Metadata Validation
- **meta.txt**: Checks for presence and validity
- **Service type**: Validates service (develop/deploy)
- **Project name**: Verifies project identifier
- **Timestamp**: Validates archive creation time

### Archive Integrity
- **TAR file**: Checks Docker image archive exists
- **File structure**: Validates expected directory structure
- **Permissions**: Ensures files are accessible

### Docker Image Validation
- **Image format**: Validates TAR archive format
- **Image metadata**: Checks Docker image metadata
- **Compatibility**: Verifies image compatibility

## Loading Process

### Step-by-Step Process

1. **Path Resolution**: Determines save directory path
2. **Validation**: Validates archive structure and metadata
3. **Image Loading**: Loads Docker image into local registry
4. **Project Setup**: Configures project environment
5. **Navigation**: Changes to appropriate directory
6. **Completion**: Reports successful loading

### Progress Indicators

```bash
$ dna load dna-save-deploy-project-202312151430
[INFO] Loading DNA project from: dna-save-deploy-project-202312151430
[INFO] Validating archive structure...
[INFO] Loading Docker image: project-deploy.latest.tar
[INFO] Configuring project environment...
[INFO] Changing to project directory: project/
[DONE] Successfully loaded deploy service
```

## Troubleshooting

### Save Directory Not Found

**Problem**: "Save directory does not exist" error.

**Solutions**:
1. **Check path**: Verify the save directory path is correct
   ```bash
   ls -la /path/to/save/directory
   ```

2. **Use absolute path**: Provide full path to save directory
   ```bash
   dna load /full/path/to/dna-save-*
   ```

### Invalid Save Directory

**Problem**: "Invalid save directory: meta.txt not found" error.

**Solutions**:
1. **Check structure**: Verify save directory structure
   ```bash
   ls -la dna-save-*/
   ```

2. **Re-create archive**: If corrupted, re-create with `dna save`
   ```bash
   # On source machine
   dna save /new/location service
   ```

### Docker Image Load Failure

**Problem**: Docker image fails to load.

**Solutions**:
1. **Check Docker**: Ensure Docker is running
   ```bash
   docker ps
   ```

2. **Check disk space**: Verify sufficient disk space
   ```bash
   df -h
   docker system df
   ```

3. **Validate archive**: Check TAR file integrity
   ```bash
   tar -tf dna-save-*/project-*.tar | head
   ```

### Permission Issues

**Problem**: Permission denied during loading.

**Solutions**:
1. **Check file permissions**: Verify archive permissions
   ```bash
   ls -la dna-save-*/*
   ```

2. **Fix permissions**: Correct file permissions if needed
   ```bash
   chmod -R u+r dna-save-*
   ```

### Project Directory Conflicts

**Problem**: Project directory already exists.

**Solutions**:
1. **Backup existing**: Move existing project
   ```bash
   mv existing-project existing-project.backup
   dna load dna-save-*
   ```

2. **Use different location**: Load to different directory
   ```bash
   mkdir /new/location
   cp -r dna-save-* /new/location/
   cd /new/location
   dna load
   ```

## Integration with Save

### Complete Save/Load Workflow

```bash
# Source machine
dna build deploy
dna save /transfer deploy

# Transfer files (USB, network, etc.)
# Target machine
dna load /transfer/dna-save-deploy-*
dna up deploy
```

### Version Management

```bash
# Save specific versions
git checkout v1.0.0
dna build deploy
dna save /archives deploy

# Load specific versions
dna load /archives/dna-save-deploy-project-v1.0.0-*
```

## Performance Considerations

- **Image size**: Loading time depends on Docker image size
- **Disk I/O**: Use fast storage for better performance
- **Network**: Local loading is faster than network storage
- **Cleanup**: Remove old images to save space

## See Also

- [dna save](save.md) - Save Docker images to files
- [dna build](build.md) - Build container images
- [Installation Guide](../install.md) - Offline installation methods

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
