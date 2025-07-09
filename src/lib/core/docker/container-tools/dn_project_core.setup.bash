#!/bin/bash
# =================================================================================================
# Dockerized-NorLab project-core image setup script i.e., user configuration install steps.
# Is executed by '.dockerized_norlab/configuration/Dockerfile' in a DN project image
#
# Usage:
#   source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_project_core.setup.bash
#
# Globals:
#   Read DN_PROJECT_ALIAS_PREFIX
#   Read DN_PROJECT_USER
#   Read DN_PROJECT_USER_HOME
#   Read DN_PROJECT_UID
#   Read DN_PROJECT_GID
#   Read DN_PROJECT_PATH
#   Read DN_DEV_WORKSPACE
#   Read/write all environment variable exposed in DN at runtime
#
# =================================================================================================
set -e
pushd "$(pwd)" >/dev/null || exit 1
dna_error_prefix="\033[1;31m[DNA error]\033[0m"

# (CRITICAL) ToDo: unit-test

function dna::setup_dockerized_norlab_project() {

  # ....Check pre-conditions.......................................................................
  # Check environment variables
  {
    test -n "${DN_PROJECT_ALIAS_PREFIX:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_USER:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_USER_HOME:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_UID:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GID:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_PATH:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_DEV_WORKSPACE:?'Env variable need to be set and non-empty.'}" && \
    test -n "${DN_PROJECT_GIT_NAME:?'Env variable need to be set and non-empty.'}" ;
  } || return 1

  # Check directories exist
  {
    test -d "${DN_PROJECT_USER_HOME}" && \
    test -d "${DN_PROJECT_PATH}" ;
  } || return 1

  # ....User specific aliases......................................................................
  echo "Add project specific aliases"
    (
      echo ""
      echo "# Project specific aliases (general)"
      echo "alias dna-${DN_PROJECT_ALIAS_PREFIX:?err}-cd='cd ${DN_PROJECT_PATH:?err}'"
      echo "alias dna-${DN_PROJECT_ALIAS_PREFIX:?err}-cdd='cd ${DN_PROJECT_PATH:?err}/.dockerized_norlab'"
      echo "alias dna-${DN_PROJECT_ALIAS_PREFIX:?err}-cds='cd ${DN_PROJECT_PATH:?err}/src'"
      echo "alias dna-${DN_PROJECT_ALIAS_PREFIX:?err}-cdt='cd ${DN_PROJECT_PATH:?err}/tests'"
      echo "alias dna-${DN_PROJECT_ALIAS_PREFIX:?err}-cda='cd ${DN_PROJECT_PATH:?err}/artifact'"
      echo "alias dna-${DN_PROJECT_ALIAS_PREFIX:?err}-cde='cd ${DN_PROJECT_PATH:?err}/external_data'"
      echo ""
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash

  # ....Entrypoint related setup...................................................................
  # Notes:
  #   - All files from 'configuration/project_entrypoints/' directory that follow the pattern
  #     'dn_entrypoint.*.callback.bash' are required by DN scripts 'dn_entrypoint.init.bash' and
  #     'dn_entrypoint.attach.bash'.
  #   - Be advised that 'project-develop' container mount this directory as a volume to prevent
  #     image rebuilding each time its content is modified (See the 'services.volumes' key in
  #     'docker-compose.project.run.*.yaml').
  #   - However, 'project-deploy' and 'project-release' containers copy this directory and its
  #     contents in the image at build time to ensure portability.
  #
  cd /project_entrypoints || return 1
  {
    test -d project-ci-tests/ && \
    test -d project-ci-tests/test_jobs && \
    test -f project-ci-tests/dn_entrypoint.init.callback.bash && \
    test -d project-slurm/ && \
    test -f project-slurm/dn_entrypoint.init.callback.bash && \
    test -d project-deploy/ && \
    test -f project-deploy/dn_entrypoint.attach.callback.bash && \
    test -f project-deploy/dn_entrypoint.init.callback.bash && \
    test -d project-develop/ && \
    test -f project-develop/dn_entrypoint.attach.callback.bash && \
    test -f project-develop/dn_entrypoint.init.callback.bash && \
    test -f dn_entrypoint.global.attach.callback.bash && \
    test -f dn_entrypoint.global.init.callback.bash ;
  } || { echo -e "${dna_error_prefix} Missing super project configuration file or directory in .dockerized_norlab/configuration/" && return 1 ; }

  for each_file in ./dn_entrypoint.*.bash; do
    chmod +x "${each_file}"
  done

  for each_file in ./project-*/dn_entrypoint.*.bash; do
    chmod +x "${each_file}"
  done

  # ....Hack.......................................................................................
  # For Matplotlib default backend (QtAgg) when python script are launched from terminal
  mkdir -m 0700 -p "/tmp/runtime-root" && chown -R "${DN_PROJECT_USER}" "/tmp/runtime-root"

  # ....Install requirement from file................................................................

  # Doc › pip install flag: https://pip.pypa.io/en/stable/cli/pip_install/#options
  pip3 install --verbose -r /python.requirements.txt || return 1

  source /shell.requirements.bash || return 1

  # ....Teardown...................................................................................
  rm -f /python.requirements.txt
  rm -f /shell.requirements.bash

  return 0
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  echo -e "${dna_error_prefix} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dna::setup_dockerized_norlab_project || exit 1
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
