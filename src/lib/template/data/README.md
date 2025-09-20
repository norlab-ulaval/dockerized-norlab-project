# Data directory

## Data Directories Purposes

Use for input source such as  _test data_, _demo data_, _experimental data_ and mounted _local data volume_.

## It contains three directories each configured for a distinct purposes

1. [External Data Directory](external_data/README.md): Non-tracked data not required by src/tests code logic
2. [Repository Data Directory](repository_data/README.md): Data that are required by the src/test code logic
3. [Shared Data Directory](shared_data/README.md): Placeholder directory replaced by an optional local data volume

# About Data And Artifact Directories

The `data/` sub-directories are configured for handling input data such as _test data_, _demo data_, _experimental data_
and mounted _local data volume_ while the `artifact/` directory is configured for handling output data such as log, plot
and trained model.

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

- [Artifact Directory README](../artifact/README.md)
- [DNA Documentation](https://github.com/norlab-ulaval/dockerized-norlab-project.git) section on _Project
  Initialization & Configuration / Directory Structure_ for details.
