#!/bin/bash
# =================================================================================================
# Check that Dockerized-Norlab-Project components and dependencies are installed at the required
# location in the super-project
#
# Usage:
#   $ bash validate_DN_project_setup.bash
#
# =================================================================================================
#
# File configuration
#
PROJECT_ENV_N2ST_FILE=".env.$( basename $(git rev-parse --show-toplevel) .git )"
#
#
function dnp::validate_DN_project_setup() {
  # ....Pre-condition..............................................................................
  # (CRITICAL) ToDo: NMO-615 feat: add logic auto set PROJECT_ENV_N2ST_FILE
  SUPER_PROJECT_ROOT=$(git rev-parse --show-toplevel)
  cd "${SUPER_PROJECT_ROOT}" || exit 1

  if [[ ! -f "$PROJECT_ENV_N2ST_FILE" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} '$PROJECT_ENV_N2ST_FILE' root directory unreachable!\n Current working directory is '$(pwd)'" 1>&2
    exit 1
  fi

  # ====Begin======================================================================================
  if [[ ! -d ".dockerized_norlab_project" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} '.dockerized_norlab_project' is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  if [[ ! -f ".dockerignore" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} '.dockerignore' is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  if [[ ! -f "$PROJECT_ENV_N2ST_FILE" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} '$PROJECT_ENV_N2ST_FILE'  is not installed at super-project repository root as it should!" 1>&2
    exit 1
  fi

  if [[ ! -d "utilities/norlab-shell-script-tools" ]]; then
    echo -e "\n${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} 'norlab-shell-script-tools' repository is not installed as a git submodule in the 'utilities/' directory as it should!" 1>&2
    exit 1
  fi

  echo "[DN-project done] Installation › OK"
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
MSG_ERROR_FORMAT="\033[1;31m"
MSG_END_FORMAT="\033[0m"
pushd "$(pwd)" >/dev/null || exit 1

dnp::validate_DN_project_setup

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
