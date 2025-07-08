# dna project

Super project DNA configuration management commands.

## Synopsis

```bash
dna project [validate|sanity|dotenv] [OPTIONS]
```

## Description

The `dna project` command provides utilities for managing and validating DNA project configurations. It includes tools for validating Docker configurations, checking project setup, and inspecting environment variable configurations.

## Subcommands

| Subcommand | Description |
|------------|-------------|
| `validate` | Validate super project setup and Docker configurations |
| `sanity` | Validate super project setup |
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
