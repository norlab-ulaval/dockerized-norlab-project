# dna run

Run commands in uniquely identified containers.

## Synopsis

```bash
# Interactive containers
dna run [OPTIONS] develop|deploy [--] [COMMAND [ARGS...]]

# Non-interactive containers  
dna run [OPTIONS] ci-tests [COMMAND [ARG...]]
dna run [OPTIONS] slurm <sjob-id> [--] <python-cmd-args>

# Generate Apptainer artifacts for HPC servers (does NOT execute apptainer locally)
dna run [OPTIONS] slurm <sjob-id> --generate-apptainer <profile> [--] <python-cmd-args>
dna run [OPTIONS] slurm <sjob-id> --ga <profile> [--] <python-cmd-args>
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
| `--log-name <name>` | Log file name without postfix |
| `--log-path <path>` | Absolute path to SLURM log directory |
| `--skip-core-force-rebuild` | Skip automatic core image rebuild |
| `--hydra-dry-run` | Dry-run SLURM job using registered hydra flag |
| `--register-hydra-dry-run-flag` | Hydra flag used by '--hydra-dry-run' |
| `--generate-apptainer`, `--ga` `<profile>` | Generate Apptainer exec script for HPC servers using Apptainer (e.g., `valeria`, `compute_canada`). **Does not execute `apptainer` locally** (macOS compatible). See `--help-slurm-apptainer`. |
| `--sif-path <path>` | (with `--generate-apptainer`) Path to SIF file on HPC server |
| `--output-dir <path>` | (with `--generate-apptainer`) Output directory for generated run script |
| `--print-only` | (with `--generate-apptainer`) Print apptainer exec command to stdout only |
| `--log-name <name>` | (with `--generate-apptainer`) Log file name for script header comment |

### Help Options

| Option | Description |
|--------|-------------|
| `--help-develop` | Show run develop help message |
| `--help-deploy` | Show run deploy help message |
| `--help-slurm` | Show run SLURM help message |
| `--help-slurm-apptainer` | Show run SLURM Apptainer help message |
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
dna run --log-name "training" --log-path "/logs" slurm job-002 -- python3 experiment.py

# Dry run SLURM job
dna run --hydra-dry-run slurm job-003 -- python3 simulation.py
```

### SLURM Jobs with Apptainer (HPC servers — Valeria, Compute Canada)

```bash
# Generate standalone Apptainer run script (does NOT execute apptainer locally)
dna run slurm NMO-001 --generate-apptainer valeria -- launcher/train.py --epochs=10
# → Creates: artifact/apptainer/run_apptainer_NMO-001.sh

# Same command using shorthand flags
dna run slurm NMO-001 --ga valeria -- launcher/train.py --epochs=10

# Print the apptainer exec command without writing a file
dna run slurm NMO-001 --ga valeria --print-only -- launcher/train.py

# Use a custom SIF path on the HPC server
dna run slurm NMO-002 --ga compute_canada \
    --sif-path /scratch/user/project/my-project.sif \
    -- launcher/experiment.py --config cfg/base.yaml
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
   dna project dotenv | grep DN_PROJECT_USER
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

## Apptainer / HPC Workflow

For HPC servers using Apptainer (Valeria, Compute Canada, Mamba), `dna run slurm <sjob-id> --generate-apptainer <profile>` (or `--ga`)
**generates** a standalone run script — it never executes `apptainer` locally (macOS compatible).

The generated run script is a **convenience/automation** tool for quick one-off runs.
For production jobs, use the **slurm job templates** (`slurm_job.apptainer.<profile>.template.bash`)
which include `#SBATCH` directives and setup/teardown hooks — submit via `sbatch` on the HPC server.
Both artifacts source the same HPC profile dotenv (`.env.<profile>`) and use the same `apptainer exec` flags.

| HPC Server | Profile | Method |
|------------|---------|--------|
| NorLab Mamba | _(none)_ | `dna run slurm` directly (Docker workflow) |
| NorLab Mamba | `mamba` | `dna run slurm --ga mamba` → generated standalone script (Apptainer workflow) |
| Ulaval Valeria | `valeria` | `dna run slurm --ga valeria` → generated standalone script |
| Compute Canada | `compute_canada` | `dna run slurm --ga compute_canada` → generated standalone script |

> **Note:** Mamba supports both Docker and Apptainer workflows. See [Apptainer / HPC Workflow](apptainer.md) for details.

See [Apptainer / HPC Workflow](apptainer.md) for the complete guide including the [three pipeline artifacts](apptainer.md#understanding-the-apptainer-pipeline-artifacts).

## See Also

- [dna up](up.md) - Start persistent containers
- [dna exec](exec.md) - Execute commands in running containers
- [dna build](build.md) - Build container images
- [Apptainer / HPC Workflow](apptainer.md) - Deploy slurm jobs on Apptainer HPC servers
- [Project Initialization & Configuration](../project_initialization_and_configuration.md) - Container configuration

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
