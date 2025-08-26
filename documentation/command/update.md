# dna update

Update Dockerized-NorLab project application (DNA) to the latest release version.

## Synopsis

```bash
dna update [OPTIONS]
```

## Description

The `dna update` command manages DNA repository updates by fetching the latest release version from the remote repository and updating the local installation. It provides intelligent update management with configurable auto-update behavior and user confirmation options.

The command compares the local DNA version with the remote repository's latest release tag and performs updates when newer versions are available.

## Options

| Option | Description |
|--------|-------------|
| `-y`, `--yes` | Auto update DNA without confirmation prompt |
| `--status` | Show update information and exit |
| `--toggle-auto` | Enable/disable daily auto-update capability |
| `--help`, `-h` | Show help message and exit |

## Examples

### Basic Update Check and Update

```bash
# Check for updates and update with user confirmation
dna update

# Force update without confirmation
dna update --yes
dna update -y
```

### Auto-Update Configuration

```bash
# Toggle auto-update setting (switches between true/false)
dna update --toggle-auto

# Shows current value and toggles to opposite state in .env.dockerized-norlab-project.local
```

### Typical Update Scenarios

```bash
# Check if update is available (with manual confirmation)
dna update

# Update immediately without prompts
dna update --yes

# Toggle auto-update setting and perform update if needed
dna update --toggle-auto && dna update
```

## What it does

When you run `dna update`, the command will:

1. **Fetch remote information**: Connects to the DNA repository and fetches the latest release tags
2. **Compare versions**: Compares local DNA version (`$DNA_VERSION`) with the latest remote release version
3. **Check auto-update setting**: Reads `DNA_AUTO_UPDATE` value from `.env.dockerized-norlab-project.local`
4. **Perform update logic**: Based on settings and flags, either updates automatically, prompts user, or skips update

## Update Behavior

### Automatic Update (DNA_AUTO_UPDATE=true)
```bash
# When auto-update is enabled, updates happen automatically
dna update
# → Checks for update
# → If available: Updates automatically without prompt
# → If up-to-date: Shows "already up to date" message
```

### Manual Update (DNA_AUTO_UPDATE=false or not set)
```bash
# When auto-update is disabled, user confirmation is required
dna update
# → Checks for update
# → If available: Prompts "Would you like to update DNA now? [y/N]:"
# → If up-to-date: Shows "already up to date" message
```

### Force Update
```bash
# Override any setting and update without confirmation
dna update --yes
# → Checks for update
# → If available: Updates immediately without prompt
```

## Configuration Management

### Toggling Auto-Update

The `--toggle-auto` flag toggles automatic update configuration:

```bash
# Toggle auto-update setting (switches between true/false)
dna update --toggle-auto
```

This command:
- Shows the current `DNA_AUTO_UPDATE` value
- Creates `.env.dockerized-norlab-project.local` if it doesn't exist
- Toggles `DNA_AUTO_UPDATE` between `true` and `false` in the file
- If unset (defaults to false), toggles to `true`
- Future `dna update` commands will behave according to the new setting

### Manual Configuration

You can also manually edit `.env.dockerized-norlab-project.local`:

```bash
# Enable auto-update
echo "DNA_AUTO_UPDATE=true" >> .env.dockerized-norlab-project.local

# Disable auto-update
echo "DNA_AUTO_UPDATE=false" >> .env.dockerized-norlab-project.local
```

## Version Management

### Update Check Logic

The update system uses semantic versioning comparison:

- **Up to date**: Local version equals remote version
- **Update available**: Local version is older than remote version  
- **Newer local version**: Local version is newer than remote (development/unreleased version)

### Version Sources

- **Local version**: Retrieved from `$DNA_VERSION` environment variable
- **Remote version**: Retrieved from latest git tag in the DNA repository
- **Version format**: Supports semantic versioning (e.g., `1.2.3`, `v1.2.3`)

## Use Cases

### Development Workflow
```bash
# Regular update check as part of daily workflow
dna update

# Quick update for urgent fixes
dna update --yes
```

### Automated Environments
```bash
# Toggle auto-update for CI/CD or automated deployments
dna update --toggle-auto

# Auto-update will now work in scripts (if toggled to true)
dna update  # Updates automatically without user intervention
```

### System Administration
```bash
# Check current version before update
dna version

# Perform update
dna update --yes

# Verify updated version
dna version
```

## Troubleshooting

### Common Issues

**Network connectivity**: Update requires internet access to fetch remote repository information.

**Git repository state**: Ensure the DNA installation is a proper git repository with remote origin configured.

**Permission issues**: Update requires write permissions to the DNA installation directory.

### Error Messages

- `"Failed to fetch remote tags from origin"`: Check network connectivity and repository access
- `"Could not determine latest remote version"`: Remote repository may not have proper version tags
- `"Failed to update DNA repository"`: Check file permissions and git repository state

### Recovery

If an update fails:

```bash
# Check current repository state
cd $DNA_ROOT
git status

# Reset to known good state if needed
git checkout main
git reset --hard origin/main

# Try update again
dna update --yes
```

## Related Commands

- [`dna version`](version.md) - Show current DNA version information
- [`dna init`](init.md) - Initialize new DNA project (includes initial setup)

## Environment Variables

| Variable | Description |
|----------|-------------|
| `DNA_VERSION` | Current local DNA version |
| `DNA_ROOT` | Path to DNA installation directory |
| `DNA_AUTO_UPDATE` | Auto-update setting (true/false) from .env file |
