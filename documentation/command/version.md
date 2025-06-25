# dna version

Show Dockerized-NorLab Project version information.

## Synopsis

```bash
dna version [OPTIONS]
```

## Description

The `dna version` command displays the current version of the Dockerized-NorLab Project (DNA) application. This is useful for troubleshooting, compatibility checking, and ensuring you're running the expected version.

## Options

| Option | Description |
|--------|-------------|
| `--help`, `-h` | Show help message and exit |

## Examples

### Basic Version Display

```bash
# Show DNA version
dna version
```

### Output Example

```bash
$ dna version
Dockerized-NorLab Project version: 1.2.3
```

## Use Cases

### Version Verification

```bash
# Check current DNA version
dna version

# Verify version in scripts
DNA_VERSION=$(dna version | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+')
echo "Running DNA version: $DNA_VERSION"
```

### Compatibility Checking

```bash
# Check version before running commands
dna version
dna build develop  # Ensure compatibility
```

### Troubleshooting

```bash
# Include version in bug reports
echo "DNA Version: $(dna version)"
echo "Docker Version: $(docker --version)"
echo "System: $(uname -a)"
```

### CI/CD Integration

```bash
# Log version in CI pipeline
echo "=== Environment Information ==="
dna version
docker --version
docker-compose --version
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

The version is read from:
- **File**: `${DNA_ROOT}/version.txt`
- **Location**: DNA installation directory
- **Format**: Plain text file containing version string

## Integration with Other Tools

### Script Integration

```bash
#!/bin/bash
# Check DNA version compatibility
REQUIRED_VERSION="1.2.0"
CURRENT_VERSION=$(dna version | grep -o '[0-9]\+\.[0-9]\+\.[0-9]\+')

if [[ ${CURRENT_VERSION} -lt ${REQUIRED_VERSION} ]]; then
    echo "Error: DNA version ${CURRENT_VERSION} is too old. Required: ${REQUIRED_VERSION}"
    exit 1
fi
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

**Problem**: Cannot read version file.

**Solutions**:
1. **Check permissions**: Verify file permissions
   ```bash
   ls -la ${DNA_ROOT}/version.txt
   ```

2. **Fix permissions**: Correct file permissions if needed
   ```bash
   chmod 644 ${DNA_ROOT}/version.txt
   ```

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
