
# Dockerized-NorLab-Project as an App: Proposed Solutions

This document outlines two approaches to convert the Dockerized-NorLab-Project from a collection of bash scripts to a more app-like structure that can be installed system-wide and called using commands similar to Docker and docker-compose.

## Current Structure Analysis

The current Dockerized-NorLab-Project is organized as follows:

```
.dockerized_norlab_project/
├── README.md
├── configuration/
│   ├── .env
│   ├── Dockerfile.*
│   ├── docker-compose.*.yaml
│   ├── project-ci-tests/
│   ├── project_entrypoints/
│   └── project_requirements/
├── dn_container_env_variable/
├── execute/
│   ├── build.all.bash
│   ├── build.deploy.bash
│   ├── down.bash
│   ├── up_and_attach.bash
│   └── ... (other execution scripts)
├── python_dev_tools/
├── utilities/
│   ├── import_dnp_lib.bash
│   ├── setup_host_for_running_this_super_project.bash
│   └── validate_super_project_dnp_setup.bash
└── visual/
```

Users currently interact with the system by:
1. Directly executing bash scripts in the `execute/` directory
2. Using aliases that point to these scripts


## Approach 1: System-Wide Installable Application

This approach transforms Dockerized-NorLab-Project into a proper system-wide installable application with a command-line interface.

<details>
  <summary style="font-weight: normal;font-size: large;">show</summary>


### Implementation Plan

1. **Create a Python Package Structure**

```
dockerized-norlab-project/
├── README.md
├── setup.py
├── pyproject.toml
├── dockerized_norlab_project/
│   ├── __init__.py
│   ├── cli.py
│   ├── commands/
│   │   ├── __init__.py
│   │   ├── build.py
│   │   ├── up.py
│   │   ├── down.py
│   │   └── ... (other commands)
│   ├── core/
│   │   ├── __init__.py
│   │   ├── docker_utils.py
│   │   ├── config_manager.py
│   │   └── ... (other core modules)
│   └── templates/
│       ├── Dockerfile.*
│       ├── docker-compose.*.yaml
│       └── ... (other templates)
├── docker/
│   └── ... (docker-related files)
└── tests/
    └── ... (test files)
```

2. **Create a Command-Line Interface**

Use [_click_](https://click.palletsprojects.com/en/stable/) or _argparse_ to create a CLI that mimics the current bash script functionality:

```python
# dockerized_norlab_project/cli.py
import click
from dockerized_norlab_project.commands import build, up, down

@click.group()
def cli():
    """Dockerized-NorLab-Project CLI"""
    pass

cli.add_command(build.command)
cli.add_command(up.command)
cli.add_command(down.command)

if __name__ == "__main__":
    cli()
```

3. **Implement Command Modules**

Each command would implement the functionality of the corresponding bash script:

```python
# dockerized_norlab_project/commands/build.py
import click
from dockerized_norlab_project.core import docker_utils

@click.command("build")
@click.option("--multiarch", is_flag=True, help="Build for multiple architectures")
@click.option("--service", help="Specify service to build")
def command(multiarch, service):
    """Build Docker images for the project"""
    docker_utils.build_images(multiarch=multiarch, service=service)
```

4. **Setup Installation**

Configure `setup.py` to install the CLI command:

```python
from setuptools import setup, find_packages

setup(
    name="dockerized-norlab-project",
    version="0.1.0",
    packages=find_packages(),
    include_package_data=True,
    install_requires=[
        "click",
        "docker",
        "pyyaml",
    ],
    entry_points="""
        [console_scripts]
        dnp=dockerized_norlab_project.cli:cli
    """,
)
```

5. **User Configuration**

Create a configuration system that allows users to customize their project:

```
~/.config/dnp/
├── config.yaml
└── templates/
    └── ... (user-customized templates)
```

### Usage Example

After installation, users would interact with the system like this:

```bash
# Initialize a new project
dnp init my-project

# Build Docker images
dnp build --multiarch

# Start and attach to a container
dnp up --service develop

# Stop containers
dnp down
```

</details>  


## Approach 2: PATH-Accessible Bash Script Approach

This approach maintains the current bash script structure but makes it accessible from anywhere via the system PATH.

<details>
  <summary style="font-weight: normal;font-size: large;">show</summary>

### Requirements:
- [ ] path management
    - Case 1 › system wide:
      - via symlink `/usr/local/bin/dnp` → `/path/to/dockerized-norlab-project/src/bin/dnp`;
      - via `~/.bashrc` ← `PATH=${PATH}:${DNP_PATH}:${NBS_PATH}:${N2ST_PATH}`.
    - Case 2 › manual load: 
      - each super project can use optionally the env var `DNP_PATH`, `NBS_PATH` and `N2ST_PATH` define in `.env.super-project-name`.
- [ ] `python_dev_tools` package
  - [ ] rename to `dnp_dev_tools` 
  - [ ] move to `src/tools` (will go in repo `redleader962-research-codebase-tools`)
- [ ] a main entrypoint: `dnp` with command (`build`, `up`, `down`, `init`) linked to corresponding bash script
  - [ ] `dnp` need to executable 
  - [ ] `dnp` need to be in `PATH`
  - [ ] `dnp init` command to initialize a project, i.e., create the super project config files, (optional) register existing project
  - [ ] `dnp update` command to check DNP version, update cloned repo and execute a per version update script to modify project DNP config
  - [ ] a project dnp config discovery mechanism so that we can do `cd path/to/repo/ && dnp build` instead of `cd path/to/repo/ && dnp build path/to/repo/path/to/config`
  - [ ] `dnp config` command to check DNP project config
  - [ ] a DNP `installer.bash` with:
    - [ ] path install option step: system wide, add to `~/.bashrc` or skip
    - [ ] install *dockerized-norlab* requirement
    - [ ] a `--yes` flag
- [ ] `.env.super-project-name`
  - [ ] `DNP_VERSION` env variable set in each dnp project to validate compatibility
  - [ ] `DNP_PATH`, `NBS_PATH` and `N2ST_PATH` as a fallback for handling local install case
- [ ] a vagrant workflow for UX development
- [ ] N2ST bats testing tools 

### Implementation Plan

1. **Create a Standalone Package Structure**

```
user-super-project/
├── .dockerized_norlab_project
│   ├── README.md
│   ├── configuration/
│   │   ├── .env # This is the user side customizable DNP environment var
│   │   ├── project-ci-tests/
│   │   │   ├── dn_entrypoint.ci_test.bash
│   │   │   ├── run_ci_tests.pytest_main.bash
│   │   │   └── run_ci_tests.pytest_only_selected.bash
│   │   ├── project_entrypoints/
│   │   │   ├── dn_entrypoint.global.attach.callback.bash
│   │   │   ├── dn_entrypoint.global.init.callback.bash
│   │   │   ├── project-deploy/
│   │   │   └── project-develop/
│   │   ├── project_requirements/
│   │   │   ├── python.requirements.txt
│   │   │   └── shell.custom_install.bash
│   │   ├── Dockerfile.project
│   │   ├── docker-compose.project.build.multiarch.yaml
│   │   ├── docker-compose.project.build.native.yaml
│   │   ├── docker-compose.project.run.darwin.yaml
│   │   ├── docker-compose.project.run.jetson.yaml
│   │   ├── docker-compose.project.run.linux-x86.yaml
│   │   └── docker-compose.project.run.slurm.yaml
│   └── dn_container_env_variable/ # Refenced by ignore files
│       └── .env.dn_expose_user_super_project # auto generated
├── src/
├── tests/
└── .env.user-super-project # generated by N2ST
```

```
dockerized-norlab-project/ # (standalone version)
├── src/
│   ├── bin/
│   │   ├── dnp # symlink to /usr/local/bin/dnp
│   │   └── dnp-completion.bash
│   ├── lib/ 
│   │   ├── commands/
│   │   │   ├── build.bash
│   │   │   ├── up.bash
│   │   │   ├── down.bash
│   │   │   ├── init.bash # project initialization
│   │   │   └── ... (other command scripts)
│   │   ├── core/
│   │   │   ├── execute/
│   │   │   │   ├── build.all.bash
│   │   │   │   ├── build.all.multiarch.bash
│   │   │   │   ├── down.bash
│   │   │   │   ├── build.all.bash
│   │   │   │   ├── build.all.multiarch.bash
│   │   │   │   ├── build.ci_tests.bash
│   │   │   │   ├── build.ci_tests.multiarch.bash
│   │   │   │   ├── build.deploy.bash
│   │   │   │   ├── build.develop.bash
│   │   │   │   ├── down.bash
│   │   │   │   ├── dryrun_and_config_test.all.bash
│   │   │   │   ├── dryrun_and_config_test.slurm.bash
│   │   │   │   ├── run.ci_tests.bash
│   │   │   │   ├── run.slurm.bash
│   │   │   │   ├── run_kill.slurm.bash
│   │   │   │   └── up_and_attach.bash
│   │   │   ├── utils/
│   │   │   │   ├── import_dnp_lib.bash
│   │   │   │   ├── setup_host_for_running_this_super_project.bash
│   │   │   │   ├── validate_super_project_dnp_setup.bash
│   │   │   │   ├── execute_compose.bash
│   │   │   │   └── dn_entrypoint.python.bash
│   │   │   └── ... (other core scripts)
│   │   ├── docker/
│   │   │   ├── .env # This is the docker compose environment varariable file specific to DNP
│   │   │   ├── Dockerfile._project
│   │   │   ├── Dockerfile._ci-tests.multiarch
│   │   │   ├── Dockerfile._ci-tests.native
│   │   │   ├── Dockerfile._run-slurm
│   │   │   ├── docker-compose._project.build.multiarch.yaml
│   │   │   ├── docker-compose._project.build.native.yaml
│   │   │   ├── docker-compose._project.run.darwin.yaml
│   │   │   ├── docker-compose._project.run.jetson.yaml
│   │   │   ├── docker-compose._project.run.linux-x86.yaml
│   │   │   └── docker-compose._project.run.slurm.yaml
│   └── configuration_template/
│       ├── .env # This is the user customizable environment var
│       ├── project-ci-tests
│       │   ├── dn_entrypoint.ci_test.bash
│       │   ├── run_ci_tests.pytest_main.bash
│       │   └── run_ci_tests.pytest_only_selected.bash
│       ├── project_entrypoints
│       │   ├── dn_entrypoint.global.attach.callback.bash
│       │   ├── dn_entrypoint.global.init.callback.bash
│       │   ├── project-deploy
│       │   └── project-develop
│       ├── project_requirements
│       │   ├── python.requirements.txt
│       │   └── shell.custom_install.bash
│       ├── Dockerfile.project
│       ├── docker-compose.project.build.multiarch.yaml
│       ├── docker-compose.project.build.native.yaml
│       ├── docker-compose.project.run.darwin.yaml
│       ├── docker-compose.project.run.jetson.yaml
│       ├── docker-compose.project.run.linux-x86.yaml
│       └── docker-compose.project.run.slurm.yaml
├── tests/
│   └── ... any N2ST bats tests 
├── user_super_project_mock/
│   └── ...
├── visual/
├── utilities
│   ├── @norlab-build-system
│   └── @norlab-shell-script-tools
├── install.bash
├── README.md
├── .dockerignore
├── .gitignore
└── .env.dockerized-norlab-project ← declare DNP, N2ST and NBS path 
```

2. **Create a Main Entry Point Script**

```bash
#!/bin/bash
# bin/dnp

# Determine the installation directory
DNP_INSTALL_DIR="$(dirname "$(dirname "$(readlink -f "$0")")")"
DNP_LIB_DIR="${DNP_INSTALL_DIR}/lib"

# Source common utilities
source "${DNP_LIB_DIR}/core/common.bash"

# Parse command
COMMAND="$1"
shift

case "${COMMAND}" in
    build)
        source "${DNP_LIB_DIR}/commands/build.bash"
        dnp::build_command "$@"
        ;;
    up)
        source "${DNP_LIB_DIR}/commands/up.bash"
        dnp::up_command "$@"
        ;;
    down)
        source "${DNP_LIB_DIR}/commands/down.bash"
        dnp::down_command "$@"
        ;;
    # ... other commands
    help)
        dnp::show_help
        ;;
    *)
        echo "Unknown command: ${COMMAND}"
        dnp::show_help
        exit 1
        ;;
esac
```

3. **Implement Command Scripts**

Each command script would implement the functionality of the corresponding current bash script:

```bash
#!/bin/bash
# lib/commands/build.bash

function dnp::build_command() {
    # Parse options
    local MULTIARCH=false
    local SERVICE=""
    
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --multiarch)
                MULTIARCH=true
                shift
                ;;
            --service)
                SERVICE="$2"
                shift 2
                ;;
            # ... other options
        esac
    done
    
    # Execute build logic
    if [[ "${MULTIARCH}" == true ]]; then
        # Build for multiple architectures
        source "${DNP_LIB_DIR}/core/docker_utils.bash"
        dnp::docker_build_multiarch "${SERVICE}"
    else
        # Build for native architecture
        source "${DNP_LIB_DIR}/core/docker_utils.bash"
        dnp::docker_build_native "${SERVICE}"
    fi
}
```

4. **Installation Script**

Create an installation script that:
- Copies files to a system directory (e.g., `/usr/local/`)
- Adds the bin directory to PATH
- Sets up bash completion

```bash
#!/bin/bash
# install.sh

# Default installation directory
INSTALL_DIR="/usr/local"

# Parse options
while [[ $# -gt 0 ]]; do
    case "$1" in
        --prefix)
            INSTALL_DIR="$2"
            shift 2
            ;;
        # ... other options
    esac
done

# Create directories
mkdir -p "${INSTALL_DIR}/bin"
mkdir -p "${INSTALL_DIR}/lib/dnp"
mkdir -p "${INSTALL_DIR}/share/doc/dnp"

# Copy files
cp -r bin/* "${INSTALL_DIR}/bin/"
cp -r lib/* "${INSTALL_DIR}/lib/dnp/"
cp -r share/* "${INSTALL_DIR}/share/"

# Make scripts executable
chmod +x "${INSTALL_DIR}/bin/dnp"

echo "Dockerized-NorLab-Project installed to ${INSTALL_DIR}"
echo "Make sure ${INSTALL_DIR}/bin is in your PATH"
```


### Usage Example

After installation, users would interact with the system like this:

```bash
# Initialize a new project
cd ~/pat/to/my/project
dnp init my-project

# Build Docker images
dnp build --multiarch

# Start and attach to a container
dnp up --service develop

# Stop containers
dnp down
```

</details>  

## Comparison of Approaches

### Approach 1: System-Wide Installable Application (Python-based)

**Advantages:**
- More modern, structured approach
- Better cross-platform compatibility
- Easier to maintain and extend
- Better integration with other Python tools
- More robust error handling and logging

**Disadvantages:**
- Requires Python knowledge to maintain
- More complex to implement initially
- Requires Python dependencies

### Approach 2: PATH-Accessible Bash Script Approach

**Advantages:**
- Maintains compatibility with existing bash scripts
- Easier transition for current users
- No additional language dependencies
- Simpler implementation

**Disadvantages:**
- Less structured than a proper application
- More difficult to maintain long-term
- Limited error handling capabilities
- Less cross-platform compatibility

## Recommendation

I recommend a hybrid approach:

1. Start with Approach 2 (PATH-Accessible Bash Script) as a short-term solution to make the current scripts more accessible
2. Gradually transition to Approach 1 (System-Wide Installable Application) for a more robust, maintainable long-term solution

This allows for immediate improvement in usability while setting the stage for a more comprehensive solution in the future.

## Implementation Roadmap

### Phase 1: PATH-Accessible Bash Scripts (1-2 months)
1. Refactor current scripts to be location-independent
2. Create a main entry point script
3. Implement an installation mechanism
4. Add bash completion support
5. Create user documentation

### Phase 2: Python CLI Transition (3-6 months)
1. Create the Python package structure
2. Implement core functionality in Python
3. Create command modules that match current bash script functionality
4. Ensure backward compatibility
5. Add comprehensive testing
6. Create user and developer documentation

### Phase 3: Full Application (6-12 months)
1. Add advanced features (logging, telemetry, etc.)
2. Implement a plugin system for extensions
3. Create a web-based UI for configuration
4. Add integration with CI/CD systems
5. Implement comprehensive error handling and recovery
