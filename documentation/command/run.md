# dna run

Run commands in uniquely identified containers.

## Synopsis

```bash
# Interactive containers
dna run [OPTIONS] develop|deploy [--] [COMMAND [ARGS...]]

# Non-interactive containers  
dna run [OPTIONS] ci-tests [COMMAND [ARG...]]
dna run [OPTIONS] slurm <sjob-id> [--] <python-cmd-args>
```

## Description

The `dna run` command creates and runs new container instances with unique identifiers. Unlike `dna up` which manages persistent containers, `dna run` creates fresh container instances each time, making it ideal for testing, CI/CD, and batch processing scenarios.

## Services

### Interactive Services

| Service | Description |
|---------|-------------|
| `develop` | Run development container instance |
| `deploy` | Run deployment container instance |

### Non-Interactive Services

| Service | Description |
|---------|-------------|
| `ci-tests` | Run continuous integration tests |
| `slurm` | Run SLURM cluster job containers |

## Options

### Interactive Services (develop/deploy)

| Option | Description |
|--------|-------------|
| `-e`, `--env stringArray` | Set container environment variables |
| `-w`, `--workdir string` | Override path to working directory |
| `-T`, `--no-TTY` | Disable pseudo-TTY allocation |
| `-v`, `--volume stringArray` | Bind mount a volume |
| `--detach` | Execute COMMAND in the background |
| `--dry-run` | Dry run mode (requires --detach flag) |
| `--help`, `-h` | Show help message |

### SLURM Service Options

| Option | Description |
|--------|-------------|
| `--log-name=<name>` | Log file name without postfix |
| `--log-path=<path>` | Absolute path to SLURM log directory |
| `--skip-core-force-rebuild` | Skip automatic core image rebuild |
| `--hydra-dry-run` | Dry-run SLURM job using registered hydra flag |
| `--register-hydra-dry-run-flag` | Hydra flag used by '--hydra-dry-run' |

### Help Options

| Option | Description |
|--------|-------------|
| `--help-develop` | Show run develop help message |
| `--help-deploy` | Show run deploy help message |
| `--help-slurm` | Show run SLURM help message |
| `--help-ci-tests` | Show run ci-tests help message |

## Examples

### Interactive Development

```bash
# Run development container with bash
dna run develop

# Run specific command in development container
dna run develop -- python3 my_script.py

# Run with custom environment variables
dna run -e "DEBUG=true" -e "LOG_LEVEL=info" develop -- pytest tests/
```

### Deployment Testing

```bash
# Test deployment container
dna run deploy

# Run deployment script
dna run deploy -- ./deploy_script.sh

# Run with custom working directory
dna run --workdir "/app" deploy -- ./production_test.sh
```

### Continuous Integration

```bash
# Build CI test images first
dna build ci-tests

# Run CI tests
dna run ci-tests

# Run specific test command
dna run ci-tests pytest tests/ --junit-xml=results.xml
```

### SLURM Jobs

```bash
# Run SLURM job with job ID
dna run slurm job-001 -- python3 train_model.py --epochs 100

# Run with custom log configuration
dna run --log-name="training" --log-path="/logs" slurm job-002 -- python3 experiment.py

# Dry run SLURM job
dna run --hydra-dry-run slurm job-003 -- python3 simulation.py
```

### Advanced Examples

```bash
# Run with volume mounts
dna run -v "/host/data:/container/data" develop -- process_data.py

# Background execution
dna run --detach deploy -- long_running_service.sh

# No TTY for automated scripts
dna run -T ci-tests -- automated_test_suite.sh
```

## Container Lifecycle

### Unique Container IDs

Each `dna run` execution creates a container with a unique identifier:
- **Format**: `<DN_CONTAINER_NAME>-<UNIQUE_ID>`
- **Isolation**: Each run is completely isolated
- **Cleanup**: Containers are automatically removed after execution

### Run vs. Up Comparison

| Aspect | `dna run` | `dna up` |
|--------|-----------|----------|
| **Container lifecycle** | Create → Run → Remove | Create → Run → Persist |
| **Use case** | Testing, CI/CD, batch jobs | Development, long-running services |
| **Container reuse** | New container each time | Reuse existing container |
| **Resource usage** | Higher (new containers) | Lower (container reuse) |

## Use Cases

### Development Testing

```bash
# Test changes without affecting main development container
dna run develop -- pytest tests/

# Quick environment testing
dna run develop -- python3 -c "import sys; print(sys.version)"
```

### CI/CD Pipelines

```bash
# Automated testing pipeline
dna build ci-tests
dna run ci-tests -- pytest tests/ --cov=src/
dna run ci-tests -- flake8 src/
dna run ci-tests -- mypy src/
```

### Batch Processing

```bash
# Process multiple datasets
for dataset in dataset1 dataset2 dataset3; do
    dna run deploy -- process_dataset.py --input "$dataset"
done
```

### SLURM Integration

```bash
# In SLURM job script
#!/bin/bash
#SBATCH --job-name=dna-training
#SBATCH --time=24:00:00

dna run slurm ${SLURM_JOB_ID} -- python3 train_model.py --config config.yaml
```

## SLURM Workflow

### Job Management

1. **Container Creation**: Automatically creates SLURM-optimized container
2. **Job Monitoring**: Handles SLURM job lifecycle
3. **Cleanup**: Automatically stops container if job is cancelled
4. **Logging**: Integrated logging with SLURM job logs

### SLURM Job Script Example

```bash
#!/bin/bash
#SBATCH --job-name=dna-experiment
#SBATCH --time=12:00:00
#SBATCH --mem=32G
#SBATCH --gpus=1

# Load DNA environment
module load docker

# Run DNA SLURM job
dna run slurm ${SLURM_JOB_ID} -- python3 experiment.py \
    --data-path /shared/data \
    --output-path /shared/results \
    --gpu-enabled
```

## Troubleshooting

### Container Creation Fails

**Problem**: Container fails to start.

**Solutions**:
1. **Check images**: Ensure images are built
   ```bash
   dna build develop  # or appropriate service
   ```

2. **Verify resources**: Check available system resources
   ```bash
   docker system df
   docker system prune  # if needed
   ```

### Permission Issues

**Problem**: Permission denied in container.

**Solutions**:
1. **Check user mapping**: Verify user configuration
   ```bash
   grep SUPER_PROJECT_USER .dockerized_norlab/configuration/.env
   ```

2. **Use volume mounts carefully**: Ensure proper permissions
   ```bash
   dna run -v "/host/path:/container/path:rw" develop
   ```

### SLURM Job Issues

**Problem**: SLURM job fails or doesn't start.

**Solutions**:
1. **Check SLURM status**: Verify SLURM is available
   ```bash
   squeue  # check job queue
   sinfo   # check node status
   ```

2. **Verify job ID**: Ensure job ID is valid
   ```bash
   dna run slurm ${SLURM_JOB_ID} -- echo "Job ID: ${SLURM_JOB_ID}"
   ```

### Resource Exhaustion

**Problem**: Too many containers created.

**Solutions**:
1. **Clean up containers**: Remove old containers
   ```bash
   docker container prune
   ```

2. **Monitor usage**: Check container usage
   ```bash
   docker ps -a --format "table {{.Names}}\t{{.Status}}\t{{.CreatedAt}}"
   ```

## Performance Considerations

- **Container overhead**: Each run creates new container (higher resource usage)
- **Image caching**: Reuses existing images (faster startup)
- **Volume mounts**: Use for large datasets to avoid copying
- **Cleanup**: Containers are automatically removed after execution

## See Also

- [dna up](up.md) - Start persistent containers
- [dna exec](exec.md) - Execute commands in running containers
- [dna build](build.md) - Build container images
- [Project Initialization & Configuration](../project_initialization_and_configuration.md) - Container configuration

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
