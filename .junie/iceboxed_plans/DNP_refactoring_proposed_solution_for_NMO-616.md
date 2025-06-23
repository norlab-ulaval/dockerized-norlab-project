
# Proposed Solution for Dockerized-NorLab-Project Setup
Reference YouTrack task: [NMO-616](https://norlab.youtrack.cloud/issue/NMO-616/refactor-split-user-side-logic-from-DN-project-internal-logic) refactor: split user side logic from DN-project internal logic 

After analyzing the project requirements and the three options presented, I recommend a modified version of Option 2 as the best approach for integrating Dockerized-NorLab-Project into user projects.

## Recommended Directory Structure

```
user-super-project
├── .dockerized_norlab # User-side configuration only
│    ├── README.md
│    ├── configuration
│    │   └── ...
│    └── dn_container_env_variable
├── ...
├── utilities
│    ├── @dockerized-norlab-project # Core logic as a submodule
│    ├── @norlab-build-system # As separate submodules
│    ├── @norlab-shell-script-tools
│    └── README.md
└── .env.user-super-project
```

## Detailed Implementation Plan

### 1. Create the Dockerized-NorLab-Project Repository

- Create a new repository called `dockerized-norlab-project`
- Initialize it with the core logic from the current `.dockerized_norlab` directory, excluding user-specific configuration
- The repository structure should follow the guidelines:

```
dockerized-norlab-project
├── README.md
├── configuration.template
│   ├── .env # Template for user customizable environment var
│   ├── project-ci-tests
│   │   └── ...
│   ├── project_entrypoints
│   │   └── ...
│   ├── project_requirements
│   │   └── ...
│   ├── Dockerfile.project
│   └── docker-compose.*.yaml
├── docker
│   ├── .env # Core docker compose environment var file
│   ├── Dockerfile._*
│   └── docker-compose._*.yaml
├── execute
│   ├── build.all.bash
│   ├── up_and_attach.bash
│   └── ...
├── src
│   └── ...
├── python_dev_tools
│   └── ...
├── user_utilities
│   ├── import_dnp_lib.bash
│   ├── setup_host_for_running_this_super_project.bash
│   └── validate_super_project_dnp_setup.bash
├── utilities # repository level tools
│    ├── @norlab-build-system
│    └── @norlab-shell-script-tools
├── visual
└── .env.dockerized-norlab-project
```

### 2. Refactor the Current `.dockerized_norlab` Directory

- Keep only user-side configuration in this directory
- The refactored structure should look like:

```
.dockerized_norlab
├── README.md
├── configuration
│   ├── .env # User customizable environment var
│   ├── project-ci-tests
│   │   └── ...
│   ├── project_entrypoints
│   │   └── ...
│   ├── project_requirements
│   │   └── ...
│   ├── Dockerfile.project
│   └── docker-compose.*.yaml
└── dn_container_env_variable
    └── .env.dn_expose_user_super_project # auto generated
```

### 3. Add Dockerized-NorLab-Project as a Submodule

- Add the new `dockerized-norlab-project` repository as a submodule in the `utilities` directory
- Use the command:
```bash
git submodule add https://github.com/norlab-ulaval/dockerized-norlab-project.git utilities/dockerized-norlab-project
```

### 4. Update Scripts to Use the New Structure

#### Updated `import_dnp_lib.bash` script:

```bash
#!/bin/bash
# =================================================================================================
# Load Dockerized-NorLab and Dockerized-NorLab-Project resources and dependencies
#
# Usage:
#   $ source import_dnp_lib.bash
#
# =================================================================================================
# Variable set for export
declare -x SUPER_PROJECT_ROOT

function dnp::import_lib_and_dependencies() {
  # ....N2ST lib configuration.......................................................................
  SUPER_PROJECT_META_DNP_DOTENV=".env.$( basename $(git rev-parse --show-toplevel) .git )"
  test -n ${SUPER_PROJECT_META_DNP_DOTENV:?'Env variable need to be set by Dockerized-NorLab installer.'} || exit 1
  
  # ....Setup......................................................................................
  local TMP_CWD
  TMP_CWD=$(pwd)
  
  # ....Pre-condition..............................................................................
  SUPER_PROJECT_ROOT=$(git rev-parse --show-toplevel)
  export SUPER_PROJECT_ROOT
  if [[ ! -f "${SUPER_PROJECT_ROOT}/${SUPER_PROJECT_META_DNP_DOTENV:?err}" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '$SUPER_PROJECT_META_DNP_DOTENV' root directory unreachable!\n Current working directory is '$(pwd)'" 1>&2
    exit 1
  fi
  
  # ....Load project .env file for N2ST............................................................
  cd "${SUPER_PROJECT_ROOT}" || exit 1
  set -o allexport
  # shellcheck disable=SC1090
  source "${SUPER_PROJECT_META_DNP_DOTENV}" || exit 1
  set +o allexport
  
  # ....Load NBS...................................................................................
  cd "${NBS_PATH:?'Variable not set'}" || exit 1
  source "import_norlab_build_system_lib.bash" || exit 1
  
  # ....(Quickhack) Reload project .env file for N2ST..............................................
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1
  set -o allexport
  # shellcheck disable=SC1090
  source "${SUPER_PROJECT_META_DNP_DOTENV}" || exit 1
  set +o allexport
  
  # ....Load N2ST..................................................................................
  cd "${N2ST_PATH:?'Variable not set'}" || exit 1
  source "import_norlab_shell_script_tools_lib.bash" || exit 1
  
  # ....(Quickhack) Reload project .env file for N2ST..............................................
  # shellcheck disable=SC1090
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1
  set -o allexport
  source "${SUPER_PROJECT_META_DNP_DOTENV}" || exit 1
  set +o allexport
  
  # ....Load Dockerized-NorLab-Project .env file...................................................
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1
  set -o allexport
  source ".dockerized_norlab/configuration/.env" || exit 1
  set +o allexport
  
  # ....Load Dockerized-NorLab-Project core logic..................................................
  cd "${SUPER_PROJECT_ROOT:?err}/utilities/dockerized-norlab-project" || exit 1
  source "import_dockerized_norlab_project_lib.bash" || exit 1
  
  #  ....Teardown...................................................................................
  cd "${TMP_CWD}" || { echo "Return to original dir error" 1>&2 && exit 1; }
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} This script must be sourced i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  dnp::import_lib_and_dependencies
fi
```

#### Updated `validate_super_project_dnp_setup.bash` script:

```bash
#!/bin/bash
# =================================================================================================
# Check that Dockerized-Norlab-Project components and dependencies are installed at the required
# location in the super-project
#
# Usage:
#   $ bash super_project_dnp_sanity_check.bash
#
# =================================================================================================
#
# File configuration
#
SUPER_PROJECT_META_DNP_DOTENV=".env.$( basename $(git rev-parse --show-toplevel) .git )"
#
#
function dnp::validate_super_project_dnp_setup() {
  # ....Pre-condition..............................................................................
  SUPER_PROJECT_ROOT=$(git rev-parse --show-toplevel)
  cd "${SUPER_PROJECT_ROOT}" || exit 1
  if [[ ! -f "$SUPER_PROJECT_META_DNP_DOTENV" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '$SUPER_PROJECT_META_DNP_DOTENV' root directory unreachable!\n Current working directory is '$(pwd)'" 1>&2
    exit 1
  fi
  
  # ====Begin======================================================================================
  if [[ ! -d ".dockerized_norlab" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '.dockerized_norlab' is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi
  
  if [[ ! -f ".dockerignore" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '.dockerignore' is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi
  
  if [[ ! -f "$SUPER_PROJECT_META_DNP_DOTENV" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '$SUPER_PROJECT_META_DNP_DOTENV'  is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi
  
  if [[ ! -d "utilities/norlab-shell-script-tools" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} 'norlab-shell-script-tools' repository is not installed as a git submodule in the 'utilities/' directory as it should!" 1>&2
    exit 1
  fi
  
  if [[ ! -d "utilities/norlab-build-system" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} 'norlab-build-system' repository is not installed as a git submodule in the 'utilities/' directory as it should!" 1>&2
    exit 1
  fi
  
  if [[ ! -d "utilities/dockerized-norlab-project" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[DNP error]${MSG_END_FORMAT} '@dockerized-norlab-project' repository is not installed as a git submodule in the 'utilities/' directory as it should!" 1>&2
    exit 1
  fi
  
  echo -e "[DNP done] Installation › OK"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"
pushd "$(pwd)" >/dev/null || exit 1
dnp::validate_super_project_dnp_setup
# ====Teardown=====================================================================================
popd >/dev/null || exit 1
```

### 5. Create Documentation for the New Structure

Updated README.md for the `.dockerized_norlab` directory:

```markdown
# Dockerized-NorLab Project

## Overview
This directory contains the user-side configuration for Dockerized-NorLab-Project. The core logic is in the `@dockerized-norlab-project` submodule in the `utilities` directory.

## Usage
1. Setup/validate `.dockerized_norlab/configuration/.env`
2. Customize files in `.dockerized_norlab/configuration/project_requirements/`. Add
3. Customize files in `.dockerized_norlab/configuration/project_entrypoints/`. Add
   project-specific container runtime logic.
4. Customize any `Dockerfile` or `docker-compose.*.yaml` to fit your need. It should work out of
   the box for most use cases.
5. From your project `src/lib/core/execute/` directory, execute the following
    ```shell
    cd utilities/dockerized-norlab-project/execute/

    # Build your DN-project containers
    bash build.all.bash
    # Start your DN-project containers
    bash up_and_attach.bash
    # Have fun
    # When your done, execute
    bash down.bash
    ```

## Requirements:
- dependencies:
    - [Docker](https://docs.docker.com/engine/install/ubuntu/#install-using-the-repository) with
      `docker-buildx-plugin` and `docker-compose-plugin`
    - [nvidia-docker](https://github.com/NVIDIA/nvidia-docker) for GPU usage
- The directory `.dockerized_norlab` need to be at the super-project repository root
- The `@dockerized-norlab-project` submodule should be in the `utilities/` directory
- The `@norlab-build-system` and `@norlab-shell-script-tools` submodules should be in the `utilities/` directory
```

### 6. Create an Installation Script for New Projects

```bash
#!/bin/bash
# =================================================================================================
# Setup Dockerized-NorLab-Project for a new project
#
# Usage:
#   $ bash setup_dockerized_norlab_project.bash
#
# =================================================================================================

function dnp::setup_dockerized_norlab_project() {
  # ....Pre-condition..............................................................................
  if [[ ! -d ".git" ]]; then
    echo -e "\n[DNP error] This script must be run from the root of a Git repository!" 1>&2
    exit 1
  fi
  
  # ....Add submodules.............................................................................
  echo "Adding @dockerized-norlab-project as a submodule..."
  git submodule add https://github.com/norlab-ulaval/dockerized-norlab-project.git utilities/dockerized-norlab-project
  
  echo "Adding @norlab-build-system as a submodule..."
  git submodule add https://github.com/norlab-ulaval/norlab-build-system.git utilities/norlab-build-system
  
  echo "Adding @norlab-shell-script-tools as a submodule..."
  git submodule add https://github.com/norlab-ulaval/norlab-shell-script-tools.git utilities/norlab-shell-script-tools
  
  # ....Initialize submodules......................................................................
  echo "Initializing submodules..."
  
  # ( (CRITICAL) ToDo: TNP-31 feat: consider adding sparse checkout to the DN-project installer
  git submodule update --init --recursive
  
  # ....Create user-side configuration.............................................................
  echo "Creating user-side configuration..."
  mkdir -p .dockerized_norlab/configuration/project_entrypoints
  mkdir -p .dockerized_norlab/configuration/project_requirements
  mkdir -p .dockerized_norlab/dn_container_env_variable
  
  # ....Copy template files........................................................................
  echo "Copying template files..."
  cp utilities/dockerized-norlab-project/configuration.template/.env .dockerized_norlab/configuration/
  cp utilities/dockerized-norlab-project/configuration.template/project_entrypoints/* .dockerized_norlab/configuration/project_entrypoints/
  cp utilities/dockerized-norlab-project/configuration.template/project_requirements/* .dockerized_norlab/configuration/project_requirements/
  cp utilities/dockerized-norlab-project/configuration.template/Dockerfile.project .dockerized_norlab/configuration/
  cp utilities/dockerized-norlab-project/configuration.template/docker-compose.*.yaml .dockerized_norlab/configuration/
  
  # ....Create symbolic links......................................................................
  echo "Creating symbolic links..."
  ln -s utilities/dockerized-norlab-project/execute .dockerized_norlab/execute
  
  # ....Create .env file...........................................................................
  echo "Creating .env file..."
  PROJECT_NAME=$(basename $(pwd))
  cat > .env.${PROJECT_NAME} << EOL
# =================================================================================================
#
# Set project related environment variables. Those are available for convenience
#   and are also required by 'norlab-shell-script-tools' library.
#
# Usage:
#
#   Important! Source this file from '${PROJECT_NAME}' repository root
#   $ cd <path/to/${PROJECT_NAME}/>
#   $ set -o allexport && source .env.${PROJECT_NAME} && set +o allexport
#
# =================================================================================================
PROJECT_PROMPT_NAME='${PROJECT_NAME}'
NBS_SPLASH_NAME=\${NBS_SPLASH_NAME:-"\${PROJECT_PROMPT_NAME}-build-system"}

# ....Programaticaly fetch source code information.................................................
PROJECT_GIT_REMOTE_URL="\$( git remote get-url origin )"
PROJECT_GIT_NAME="\$( basename \${PROJECT_GIT_REMOTE_URL} .git )"
PROJECT_PATH="\$( git rev-parse --show-toplevel )"
PROJECT_SRC_NAME="\$( basename \${PROJECT_PATH} )"

# ....Set project related environment variable with their own prefix...............................
# Note: Those with "PROJECT_" prefix will get eventualy overiden in the case where N2ST is used
#       as a library. Using generic testing logic require that environment variables with
#       "PROJECT_" prefix be available.
${PROJECT_NAME}_SPLASH_NAME="\${PROJECT_GIT_NAME} (\${PROJECT_PROMPT_NAME})"
${PROJECT_NAME}_PROMPT_NAME="\${PROJECT_PROMPT_NAME}"
${PROJECT_NAME}_GIT_REMOTE_URL="\${PROJECT_GIT_REMOTE_URL}"
${PROJECT_NAME}_GIT_NAME="\${PROJECT_GIT_NAME}"
${PROJECT_NAME}_PATH="\${PROJECT_PATH}"
${PROJECT_NAME}_SRC_NAME="\${PROJECT_SRC_NAME}"

# ....Set dependencies path........................................................................
N2ST_PATH="\${PROJECT_PATH}/utilities/norlab-shell-script-tools"
NBS_PATH="\${PROJECT_PATH}/utilities/norlab-build-system"
DNP_PATH="\${PROJECT_PATH}/utilities/dockerized-norlab-project"
NBS_VERSION="\$(cat \${NBS_PATH}/version.txt)"
N2ST_VERSION="\$(cat \${N2ST_PATH}/version.txt)"
DNP_VERSION="\$(cat \${DNP_PATH}/version.txt)"
EOL

  # (Priority) ToDo: update all existing script to use DNP_PATH env var 
  
  # ....Create .dockerignore file..................................................................
  echo "Creating .dockerignore file in super project root ..."
  cp utilities/dockerized-norlab-project/.dockerignore .
  
  # ....Done......................................................................................
  echo "Dockerized-NorLab-Project setup complete!"
  echo "Next steps:"
  echo "1. Customize .dockerized_norlab/configuration/.env"
  echo "2. Customize files in .dockerized_norlab/configuration/project_requirements/"
  echo "3. Customize files in .dockerized_norlab/configuration/project_entrypoints/"
  echo "4. Run the following commands to build and start the Docker containers:"
  echo "   cd utilities/dockerized-norlab-project/execute/"
  echo "   bash build.all.bash"
  echo "   bash up_and_attach.bash"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp::setup_dockerized_norlab_project
```

## Advantages of This Solution

1. **Clean Separation of Concerns**:
   - User-specific configuration stays in the user project
   - Core logic is maintained in a separate repository

2. **Simplified Maintenance**:
   - Updates to the core logic can be pulled by updating the submodule
   - User-specific configurations remain untouched during updates

3. **Consistent Structure**:
   - All dependencies are in the `utilities` directory
   - Clear distinction between user configuration and core logic

4. **Easy Setup for New Projects**:
   - Installation script automates the setup process
   - Template files provide a starting point for customization

5. **Improved Collaboration**:
   - Core logic can be developed and tested independently
   - Multiple projects can use the same core logic

This implementation plan provides a clear path forward for refactoring the Dockerized-NorLab-Project into a more modular and reusable structure, making it easier to maintain and use across multiple projects.
