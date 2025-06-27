# dna version

Show Dockerized-NorLab project application version information.

## Synopsis

```bash
dna version [OPTIONS]
```

## Description

The `dna version` command displays the current version of Dockerized-NorLab project application. This is useful for troubleshooting, compatibility checking, and ensuring you're running the expected version.

## Options

| Option | Description |
|--------|-------------|
| `--short`, `-s` | Show version number only |
| `--all`, `-a` | Show detailed version information |
| `--help`, `-h` | Show help message and exit |

## Output Examples

### Default Version Display

```bash
$ dna version
Dockerized-NorLab project application version 1.2.3
```

### Short Version Display
Show only version number

```bash
$ dna version --short
1.2.3
```

### Detailed Version Display
Show comprehensive version information

```bash
dna version --all
$ dna version --all
Dockerized-NorLab project application:
  Version: 1.2.3
  Config scheme version: 1
  Submodule version:
    norlab-shell-script-tools: 2.0.0
    norlab-build-system: 3.0.0
  Local repository:
    Current branch: main
    Current commit: abc123def456
  Host architecture and OS: linux/amd64
```

## Version Information

### Version Format

DNA follows semantic versioning (SemVer):
- **Format**: `MAJOR.MINOR.PATCH`
- **Example**: `1.2.3`
  - `1` - Major version (breaking changes)
  - `2` - Minor version (new features, backward compatible)
  - `3` - Patch version (bug fixes, backward compatible)

### Version Source

The version information is obtained from:
- **Environment Variables**: DNA_VERSION, DNA_CONFIG_SCHEME_VERSION, N2ST_VERSION, NBS_VERSION
- **Git Repository**: Current branch and commit information
- **System**: Host architecture and OS information


## Use Cases

### Version Verification

```bash
# Get just the version number for scripts
$ echo "Running DNA version: $(dna version --short)"
Running DNA version: 1.2.3
```

### Compatibility Checking

```bash
# Check version before running commands
$ dna version --all | grep -e "Config scheme version"
Config scheme version: 1
$ cat .dockerized_norlab/.env.dockerized-norlab-project-mock | grep DNA_CONFIG_SCHEME_VERSION
DNA_CONFIG_SCHEME_VERSION=1
```

### CI/CD Integration

```bash
# Log version in CI pipeline
echo "=== Environment Information ==="
dna version
docker --version
docker-compose --version
```

### Troubleshooting

```bash
# Include comprehensive version information in bug reports
cat > "mock_bug_report.log" << EOF
=== DNA Version Information ===
$(dna version --all)

=== Docker Information ===
Docker Version: $(docker --version)
Docker Compose Version: $(docker-compose --version)
EOF
```

## Integration with Other Tools

### Script Integration

```bash
#!/bin/bash
# Check DNA version compatibility
REQUIRED_VERSION="1.2.0"
CURRENT_VERSION=$(dna version --short)

if [[ ${CURRENT_VERSION} < ${REQUIRED_VERSION} ]]; then
    echo "Error: DNA version ${CURRENT_VERSION} is too old. Required: ${REQUIRED_VERSION}"
    exit 1
fi

# Get detailed information for logging
echo "=== DNA Environment ==="
dna version --all
```

### Docker Image Tagging

```bash
# Use DNA version for image tagging
DNA_VERSION=$(dna version | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+')
docker tag my-project:latest my-project:dna-${DNA_VERSION}
```

### Documentation Generation

```bash
# Include version in generated documentation
echo "Generated with DNA version: $(dna version)" >> README.md
```

## Troubleshooting

### Version File Not Found

**Problem**: "Error: version.txt not found" message.

**Cause**: DNA installation is incomplete or corrupted.

**Solutions**:
1. **Reinstall DNA**: Perform a fresh installation
   ```bash
   cd /path/to/dockerized-norlab-project
   bash install.bash
   ```

2. **Check installation**: Verify DNA installation directory
   ```bash
   which dna
   ls -la $(dirname $(which dna))/../
   ```

3. **Manual verification**: Check if version.txt exists
   ```bash
   find /usr/local -name "version.txt" 2>/dev/null | grep dna
   ```

### Permission Issues

### Incorrect Version Display

**Problem**: Version shows unexpected value.

**Solutions**:
1. **Check installation**: Verify you're running the correct DNA
   ```bash
   which dna
   readlink -f $(which dna)
   ```

2. **Check version file**: Manually inspect version file
   ```bash
   cat ${DNA_ROOT}/version.txt
   # compare with 
   dna version --short
   ```

## Version History and Compatibility

### Checking Compatibility

Different DNA versions may have different features and requirements:

```bash
# Check if specific features are available
DNA_VERSION=$(dna version | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+')
case "$DNA_VERSION" in
    1.0.*) echo "Basic DNA features available" ;;
    1.1.*) echo "Enhanced build features available" ;;
    1.2.*) echo "SLURM integration available" ;;
    *) echo "Unknown version capabilities" ;;
esac
```

### Upgrade Considerations

When upgrading DNA versions:

1. **Check release notes**: Review changes and breaking changes
2. **Backup projects**: Save current project configurations
3. **Test compatibility**: Verify existing projects work with new version
4. **Update documentation**: Update project documentation if needed

## See Also

- [Installation Guide](../install.md) - DNA installation and setup
- [dna config](config.md) - Configuration information
- [dna project](project.md) - Project management commands

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
