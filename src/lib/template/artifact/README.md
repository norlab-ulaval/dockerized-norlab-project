# Artifact

## Directory purpose

Project artifact should go in here e.g., experimental log, plot, trained model, ...

## Properties:

- Vcs non-tracked data
- Rsync to remote host
- Docker read-and-write volume

## Notes

- ★ Dockerized-NorLab project application (DNA) **required** directory.
- Be advised, this directory is VCS ignored so these data need to be safeguarded on another device.
- Directory `artifact/optuna_storage/` is required by `hydra-optuna-sweeper` dna configuration for hyperparam search.

# About Artifact And Data Directories

The `artifact/` directory is configured for handling output data such as log, plot and trained model while the `data/`
sub-directories are configured for handling input data such as _test data_, _demo data_, _experimental data_ and mounted
_local data volume_

```terminaloutput
⋮
├── artifact/                           ← Runtime produced data
├── data/
│   ├── external_data/                  ← Non-tracked data not required by src/tests code logic
│   ├── repository_data/                ← Data that are required by the src/test code logic
│   └── shared_data/                    ← Placeholder directory replaced by an optional local data volume
⋮
```

## Data and Artifact Directory Properties Summary

| Directory               | Purpose                         | Docker Mount Behavior   | Version Control System Behavior | Remote Development |
|-------------------------|---------------------------------|-------------------------|---------------------------------|--------------------|
| `artifact/`             | Runtime data (i.e., output)     | Persistent volume mount | VCS Ignored                     | Rsync              |
| `data/`                 | Input Data                      |                         |                                 |                    |
| `data/external_data/`   | External data                   | Read-and-write mount    | VCS Ignored                     | Rsync              |
| `data/repository_data/` | Source/tests code required data | Read-and-write mount    | VCS Tracked                     | Rsync              |
| `data/shared_data/`     | External data                   | Read-only mount         | VCS Ignored                     | Local only         |

## See Also

- [Data Directories README](../data/README.md)
- [External Data Directory README](../data/external_data/README.md)
- [Repository Data Directory README](../data/repository_data/README.md)
- [Shared Data Directory README](../data/shared_data/README.md)
- [DNA Documentation](https://github.com/norlab-ulaval/dockerized-norlab-project.git) section on _Project
  Initialization & Configuration / Directory Structure_ for details.
