#!/bin/bash
# =================================================================================================
# Dockerized-NorLab project-core image setup script i.e., user configuration install steps.
# Is executed by '.dockerized_norlab_project/configuration/Dockerfile' in a DN project image
#
# Usage:
#   source /dockerized-norlab/dockerized-norlab-images/container-tools/dn_project_core_setup.bash
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

# (CRITICAL) ToDo: unit-test

function dnp::setup_dockerized_norlab_project() {

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
      echo "alias dnp${DN_PROJECT_ALIAS_PREFIX:?err}_cd='cd ${DN_PROJECT_PATH:?err}'"
      echo "alias dnp${DN_PROJECT_ALIAS_PREFIX:?err}_cdd='cd ${DN_PROJECT_PATH:?err}/src'"
      echo "alias dnp${DN_PROJECT_ALIAS_PREFIX:?err}_cdt='cd ${DN_PROJECT_PATH:?err}/tests'"
      echo "alias dnp${DN_PROJECT_ALIAS_PREFIX:?err}_cde='cd ${DN_PROJECT_PATH:?err}/external_data'"
      echo ""
    ) >> /dockerized-norlab/dockerized-norlab-images/container-tools/dn_bash_alias.bash

  # ....Entrypoint related setup...................................................................
  # Notes:
  #   - All files from 'configuration/project_entrypoints/' directory that follow the pattern
  #     'dn_entrypoint.*.callback.bash' are required by DN scripts 'dn_entrypoint.init.bash' and
  #     'dn_entrypoint.attach.bash'.
  #   - Be advise that 'project-develop' container mount this directory as a volume to prevent
  #     image rebuilding each time its content is modified (See the 'services.volumes' key in
  #     'docker-compose.project.run.*.yaml').
  #   - However, 'project-deploy' and 'project-release' containers copy this directory and its
  #     contents in the image at build time to ensure portability.
  #
  cd /project_entrypoints || return 1
  test -d project-ci-tests/ || exit 1
  test -d project-ci-tests/test_jobs || exit 1
  test -f project-ci-tests/dn_entrypoint.ci_test.bash || exit 1
  test -d project-deploy/ || exit 1
  test -f project-deploy/dn_entrypoint.attach.callback.bash || exit 1
  test -f project-deploy/dn_entrypoint.init.callback.bash || exit 1
  test -d project-develop/ || exit 1
  test -f project-develop/dn_entrypoint.attach.callback.bash || exit 1
  test -f project-develop/dn_entrypoint.init.callback.bash || exit 1
  test -f dn_entrypoint.global.attach.callback.bash || exit 1
  test -f dn_entrypoint.global.init.callback.bash || exit 1
  test -f dn_entrypoint.python.bash || exit 1

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
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  MSG_ERROR_FORMAT="\033[1;31m"
  MSG_END_FORMAT="\033[0m"
  echo -e "${MSG_ERROR_FORMAT}[ERROR]${MSG_END_FORMAT} This script must be sourced!
        i.e.: $ source $(basename "$0")" 1>&2
  exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  dnp::setup_dockerized_norlab_project || exit 1
fi

# ====Teardown=====================================================================================
popd >/dev/null || exit 1
