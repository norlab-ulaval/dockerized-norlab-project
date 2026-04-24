# dna project

Super project DNA configuration management commands.

## Synopsis

```bash
dna project [validate|sanity|init_secrets|dotenv] [OPTIONS]
```

## Description

The `dna project` command provides utilities for managing and validating DNA project configurations. It includes tools for validating Docker configurations, checking project setup, and inspecting environment variable configurations.

## Subcommands

| Subcommand | Description |
|------------|-------------|
| `validate` | Validate super project setup and Docker configurations |
| `sanity` | Validate super project setup |
| `init_secrets` | Initialize DNA secrets for the super project |
| `dotenv` | Show consolidated and interpolated dotenv config files |

## Options

| Option | Description |
|--------|-------------|
| `--help`, `-h` | Show help message and exit |

## Subcommand Details

### validate

Validate DNA project Docker configurations including docker-compose files, dotenv files, and shell variable substitution.

#### Synopsis
```bash
dna project validate [OPTIONS]
```

#### Options
| Option | Description |
|--------|-------------|
| `--slurm ["<slurm/job/dir/path>"]` | Validate only the SLURM configuration |
| `--include-multiarch` | Include multi-architecture image validation |
| `--help`, `-h` | Show help message |

#### Notes
- When using `--slurm`, you may optionally provide the SLURM job directory path as a positional argument. If omitted, it defaults to `slurm_jobs/`.

#### What it does
1. **Configuration validation**: Checks docker-compose files with variable interpolation
2. **Build validation**: Executes build in dry-run mode for each service
3. **Run validation**: Tests container execution in dry-run mode
4. **SLURM validation**: Optionally validates SLURM-specific configurations

### sanity

Validate super project setup and configuration.

#### Synopsis
```bash
dna project sanity [OPTIONS]
```

#### Options
| Option | Description |
|--------|-------------|
| `--help`, `-h` | Show help message |

### init_secrets

Initialize DNA secrets for the super project.

Generates a strong password at `.dockerized_norlab/configuration/secrets/dna_ssh_password.txt` and ensures it is secured with the correct file ownership and permissions. If the secrets file already exists, it is left unchanged unless the `--override` flag is provided. The command also appends a `**/secrets/*` rule to the super project's `.gitignore` to prevent accidental commits of secrets (added only once).

#### Synopsis
```bash
dna project init_secrets [OPTIONS]
```

#### Options
| Option | Description |
|--------|-------------|
| `--override` | Generate a new strong password even if the file already exists (overwrites) |
| `--help`, `-h` | Show help message |

#### Notes
- Secret file path: `.dockerized_norlab/configuration/secrets/dna_ssh_password.txt`
- The secret is a random 32-byte base64 string generated with `openssl rand -base64 32`
- File permissions are validated via DNA's `validate_file_ownership_and_permissions`
- A `**/secrets/*` entry is added to the repository's `.gitignore` if not already present

#### Examples
```bash
# Initialize secrets (create file if missing)
dna project init_secrets

# Force regeneration of the secret
dna project init_secrets --override
```

### dotenv

Show consolidated and interpolated dotenv configuration files.

#### Synopsis
```bash
dna project dotenv [OPTIONS]
```

#### Options
| Option | Description |
|--------|-------------|
| `--help`, `-h` | Show help message |

#### What it shows
- **Environment file precedence**: Shows how variables are resolved
- **Variable interpolation**: Displays final values after substitution
- **Configuration sources**: Shows which files contribute to final configuration
- **Grouped output**:
  - DNA/DN variables: `DN_*`, `DNA_*`
  - Project variables: `PROJECT_*`, `SUPER_*`
  - Norlab Build System (NBS): `NBS_*`
  - Norlab Shell Script Tools (N2ST): `N2ST_*`

## Examples

### Basic Validation

```bash
# Validate entire project configuration
dna project validate

# Validate project sanity
dna project sanity

# Show environment configuration
dna project dotenv
```

### SLURM Validation

```bash
# Validate SLURM configuration with default directory
dna project validate --slurm

# Validate SLURM configuration with custom directory
dna project validate --slurm "/path/to/slurm/jobs"
```

> **Note — HPC-target apptainer scripts are skipped off-HPC.**
>
> `dna project validate --slurm` iterates over every `slurm_job.*.bash` found in
> the target directory and executes each one in dry-run mode to catch
> configuration issues early.
>
> Slurm job scripts whose filename matches `*.apptainer.*.bash` (for example
> `slurm_job.<name>.apptainer.valeria.bash`) are **standalone HPC submission
> scripts**: they are meant to be executed by `sbatch` on an HPC login/compute
> node (Valeria, etc.) where `apptainer` and `module` (Lmod) are available.
> They do **not** honor DNA's `--hydra-dry-run` / `--skip-*-force-rebuild`
> flags and they call `module` / `apptainer` unconditionally.
>
> To avoid environment-only failures when `dna project validate --slurm` runs
> on a non-HPC host (e.g. a developer laptop, a CI build agent such as
> TeamCity, etc.), any `*.apptainer.*.bash` script is **automatically skipped
> when `apptainer` is not available on the current host's `PATH`**. Skipped
> scripts are listed in the validate output so you can easily see that they
> were not dry-run. When `apptainer` **is** available on `PATH`, these scripts
> are dry-run like the others.
>
> The filter applies to **every** HPC profile (not just `valeria`): any file
> matching the glob `slurm_job.*.apptainer.*.bash` is affected.
>
> At the end of the run, a **"Skipped slurm job summary"** section is printed
> alongside the config / build / dry-run summaries, listing each skipped
> script with the reason (e.g. `apptainer` not available on this host). This
> section is only shown when at least one script was skipped.

### Multi-Architecture Validation

```bash
# Validate with multi-architecture image support
dna project validate --include-multiarch

# Combine with SLURM validation
dna project validate --include-multiarch --slurm

# SLURM validation with custom directory and multi-arch support
dna project validate  --include-multiarch --slurm "/path/to/slurm/jobs"
```

### Configuration Inspection

```bash
# Check environment variable resolution
dna project dotenv

# Validate after configuration changes
dna project validate
```

## Use Cases

### Development Workflow

```bash
# After modifying .env files
dna project dotenv          # Check variable resolution
dna project validate        # Ensure configuration is valid

# Before committing changes
dna project sanity          # Quick sanity check
dna project validate        # Full validation
```

### CI/CD Integration

```bash
# In CI pipeline
dna project validate        # Validate configuration
dna build ci-tests         # Build if validation passes
dna run ci-tests           # Run tests
```

### SLURM Setup

```bash
# Validate SLURM job configurations
dna project validate --slurm

# Check specific SLURM job directory
dna project validate --slurm "/cluster/jobs"
```

### Debugging Configuration Issues

```bash
# Check environment variable resolution
dna project dotenv

# Validate specific components
dna project validate --slurm    # SLURM-specific validation
dna project sanity              # General sanity checks
```

## Validation Process

### Configuration Validation Steps

1. **Environment loading**: Loads and validates dotenv files
2. **Variable interpolation**: Resolves all environment variables
3. **Docker Compose validation**: Validates compose file syntax and variables
4. **Service validation**: Checks each service configuration
5. **Build validation**: Tests build process in dry-run mode
6. **Runtime validation**: Tests container execution

### What Gets Validated

- **Docker Compose files**: Syntax and variable substitution
- **Environment files**: Variable resolution and precedence
- **Service definitions**: Container configurations
- **Volume mounts**: Path validation and permissions
- **Network configurations**: Port mappings and network settings
- **Build contexts**: Dockerfile and build arguments

## Environment File Precedence

The `dotenv` subcommand shows how environment variables are resolved:

1. `.env.dna` (super project) - DNA-specific settings
2. `.env` (super project) - General project settings
3. `.env.local` (super project) - Local env variables overrides
4. `.env.dna-internal` (DNA repo) - Internal DNA settings  

## Troubleshooting

### Validation Failures

**Problem**: `dna project validate` reports errors.

**Solutions**:
1. **Check environment files**: Verify syntax and variable names
   ```bash
   dna project dotenv  # Check variable resolution
   ```

2. **Validate Docker Compose**: Check compose file syntax
   ```bash
   docker-compose -f .dockerized_norlab/configuration/docker-compose.yml config
   ```

3. **Check file paths**: Ensure all referenced files exist
   ```bash
   ls -la .dockerized_norlab/configuration/
   ```

### Environment Variable Issues

**Problem**: Variables not resolving correctly.

**Solutions**:
1. **Check precedence**: Use `dotenv` to see resolution order
   ```bash
   dna project dotenv
   ```

2. **Verify file syntax**: Check for syntax errors in .env files
   ```bash
   # Check for spaces around = signs
   grep -n " = " .dockerized_norlab/configuration/.env
   ```

3. **Test interpolation**: Verify variable substitution
   ```bash
   # Check specific variable
   grep VARIABLE_NAME .dockerized_norlab/configuration/.env*
   ```

### SLURM Validation Issues

**Problem**: SLURM validation fails.

**Solutions**:
1. **Check SLURM directory**: Verify directory exists and contains job scripts
   ```bash
   ls -la slurm_jobs/
   ```

2. **Validate job scripts**: Check SLURM script syntax
   ```bash
   # Check for SBATCH directives
   grep "#SBATCH" slurm_jobs/*.bash
   ```

### Permission Issues

**Problem**: Permission denied during validation.

**Solutions**:
1. **Check file permissions**: Ensure files are readable
   ```bash
   ls -la .dockerized_norlab/configuration/
   ```

2. **Check Docker permissions**: Ensure Docker is accessible
   ```bash
   docker ps  # Should not require sudo
   ```

## Integration with Other Commands

### Workflow Integration

```bash
# Complete project setup workflow
dna init                    # Initialize project
dna project validate        # Validate configuration
dna build develop          # Build if validation passes
dna up                     # Start containers
```

### Configuration Management

```bash
# After configuration changes
dna project dotenv          # Check variable resolution
dna project validate        # Validate changes
dna build develop          # Rebuild if needed
```

## See Also

- [dna init](init.md) - Initialize DNA projects
- [dna build](build.md) - Build container images
- [Project Initialization & Configuration](../project_initialization_and_configuration.md) - Configuration guide
- [Installation Guide](../install.md) - DNA installation

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
