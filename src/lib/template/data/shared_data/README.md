# Shared Data

### Directory purpose
Placeholder directory replaced by an optional local data volume. 

### Configuration instructions
Set the target data directory path via `DNA_HOST_SHARED_DATA_PATH` environment variable in `.dockerized_norlab/configuration/.env.local`.

```dotenv
DNA_HOST_SHARED_DATA_PATH=/Path/to/host/computer/shared_data/directory
```
It will be accessible at runtime in the dna container at `data/shared_data/`.
Many container can mount the target path at the same time.

### Properties
- Its a placeholder directory
- Cannot be rsync to remote host 
- Docker read-only volume

## Notes
- ★ Dockerized-NorLab project application (DNA) **required** directory.

## See Also
 
- [Data Directories README](../README.md)
- [External Data Directory README](../external_data/README.md)
- [Repository Data Directory README](../repository_data/README.md)
- [Artifact Directory README](../../artifact/README.md)
- [DNA documentation](https://github.com/norlab-ulaval/dockerized-norlab-project?tab=readme-ov-file#documentation) on
  _Project Initialization & Configuration_, section _Directory Structure_ for details.
