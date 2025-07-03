# dna init

Initialize a new DNA project in the current repository.

## Synopsis

```bash
dna init [--help]
```

## Description

The `dna init` command sets up a new Dockerized-NorLab (DN) project by creating the necessary directory structure and configuration files in your existing Git repository. This command transforms a regular repository into a DNA-enabled project ready for containerized development.

## Requirements

- **Git repository**: Must be executed at the target project repository root (your project's top directory)
- **Version control**: The repository must be under version control using Git
- **Working directory**: Must be run from the repository root directory (where `.git` exists)

## Arguments

| Argument | Description |
|----------|-------------|
| `--help`, `-h` | Show help message and exit |

## What it does

When you run `dna init`, the command will:

1. **Validate environment**: Check that you're in a Git repository root
2. **Create directory structure**: Add the following directories to your project:
   ```
   your-project-repository/
   ├── .dockerized_norlab/             ← DNA configuration
   ├── artifact/                       ← Runtime produced data (mounted)
   ├── external_data/                  ← Pre-existing data (mounted)
   ├── src/                           ← Your source code (mounted/copied)
   ├── tests/                         ← Your test code (mounted/copied)
   ├── .dockerignore                  ← Docker build exclusions
   ├── .gitignore                     ← Git exclusions
   └── README.md                      ← Created if missing
   ```

3. **Setup configuration files**: Create and configure:
   - Environment files (`.env`, `.env.dna`, `.env.local`)
   - Docker configuration (`Dockerfile`)
   - Project-specific requirements and entrypoints
   - Container runtime configurations

4. **Preserve existing content**: Any existing directories or files will be updated, not overridden

## Examples

### Basic initialization

```bash
# Navigate to your project repository
cd /path/to/your/project

# Initialize DNA
dna init
```

### Check if already initialized

If you run `dna init` on an already initialized project, you'll be prompted:

```bash
$ dna init
[WARNING] This project is already DNA initialized since .dockerized_norlab directory already exists.
If you continue, existing file and directories with the same name will be safeguarded with the suffix '.old', not overriden."
Do you want to continue [y/N]
```

## Post-initialization

After running `dna init`, you can:

1. **Build your project**: `dna build develop`
2. **Start containers**: `dna up`
3. **Customize configuration**: Edit files in `.dockerized_norlab/configuration/`

## Notes

- The `artifact/` directory content is persistent and remains available even if containers are stopped or removed
- Configuration files in `.dockerized_norlab/` can be customized for your specific project needs
- See [Project Initialization & Configuration](../project_initialization_and_configuration.md) for detailed configuration options

## Troubleshooting

### "Cwd is not at repository root"
**Problem**: You're not in the Git repository root directory.  
**Solution**: Navigate to your project's root directory (where `.git` exists) and run the command again.

### Permission issues
**Problem**: Permission denied when creating directories.  
**Solution**: Ensure you have write permissions in the current directory.

## See Also

- [dna build](build.md) - Build DNA Docker images
- [dna up](up.md) - Start containers
- [Project Initialization & Configuration](../project_initialization_and_configuration.md) - Detailed configuration guide

## Navigation

- [← Back to Command Reference](../dna.md)
- [← Back to Main README](../../README.md)
