# External Data

## Directory purpose
Directory for non-tracked data not required by source code or tests code logic.

## Use cases: 
Temporary data that you want to use on a remote host, experimental input data, data downloaded at dna runtime e.g., from a NAS, from a www dataset 

## Directory usage examples: 
- Rsync from a data directory on host
- Use it as a target path for a dataset download script
- Simply Manualy copy files 

## Properties: 
- Vcs non-tracked data
- Rsync to remote host 
- Docker read-and-write volume

## Notes

- ★ Dockerized-NorLab project application (DNA) **required** directory.
- Be advised, this directory is VCS ignored so these data need to be safeguarded on another device.

## See Also
 
- [Data Directories README](../README.md)
- [Repository Data Directory README](../repository_data/README.md)
- [Shared Data Directory README](../shared_data/README.md)
- [Artifact Directory README](../../artifact/README.md)
- [DNA documentation](https://github.com/norlab-ulaval/dockerized-norlab-project?tab=readme-ov-file#documentation) on
  _Project Initialization & Configuration_, section _Directory Structure_ for details.
