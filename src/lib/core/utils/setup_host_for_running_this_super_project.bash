#!/bin/bash
# =================================================================================================
# Setup host computer (i.e, workstation, embed computer, server) for using this
# Dockerized-NorLab-Project super project.
#
# Usage:
#   $ bash setup_host_for_running_this_super_project.bash
#   or
#   $ source setup_host_for_running_this_super_project.bash && dnp::setup_host_for_this_super_project
#
# Global:
#  read SUPER_PROJECT_ROOT
#  read SUPER_PROJECT_REPO_NAME
#  read DN_PROJECT_ALIAS_PREFIX
#
# =================================================================================================

# (Priority) ToDo: Make the script multi-arch and multi-os by calling specialize version
# see:
# - `libpointmatcher-build-system build_system/lpm_server_config.bash`
# - https://github.com/vaul-ulaval/f1tenth_controller/blob/272573b99855779428fccd122b5f84fc172e0767/setup_dockerized_norlab_for_this_repo.bash
# - https://github.com/norlab-ulaval/dockerized-norlab/blob/8975e05e69ddc57cb858a3413ea7c98920f5422b/jetson_xavier_install.bash

function dnp::setup_host_for_this_super_project() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  if [[ -z "${SUPER_PROJECT_ROOT}" ]] || [[ -z "${SUPER_PROJECT_REPO_NAME}" ]] || [[ -z "${DN_PROJECT_ALIAS_PREFIX}" ]]; then
    n2st::print_msg_error_and_exit "Super project configs are not loaded, run load_super_project_config.bash first!"
  fi

  if [[ ! -d "${SUPER_PROJECT_ROOT}" ]]; then
    n2st::print_msg_error_and_exit "The '$(basename "${SUPER_PROJECT_ROOT}")' dir is unreachable!"
  else
    cd "${SUPER_PROJECT_ROOT:?err}" || exit 1
    n2st::print_msg_done "The '$(basename "${SUPER_PROJECT_ROOT}")' dir is reachable. Ready to install alias"
  fi

  # ...aliasing dev................................................................................
  # ref:
  # - https://www.baeldung.com/linux/bash-alias-with-parameters
  # - https://unix.stackexchange.com/questions/3773/how-to-pass-parameters-to-an-alias

  local dn_project_alias_prefix_caps="$(echo "${DN_PROJECT_ALIAS_PREFIX}" | tr '[:lower:]' '[:upper:]')"
  (
    echo ""
    echo "#>>>>DNP ${SUPER_PROJECT_REPO_NAME:?err} aliases and env variable"
    echo "export _DNP_${dn_project_alias_prefix_caps}_PATH=${SUPER_PROJECT_ROOT:?err}/.dockerized_norlab_project"
    echo "alias dnp-${DN_PROJECT_ALIAS_PREFIX}-cd='cd $SUPER_PROJECT_ROOT'"
    echo "alias dnp-${DN_PROJECT_ALIAS_PREFIX}-cdd='cd ${SUPER_PROJECT_ROOT}/.dockerized_norlab_project'"
    echo "alias dnp-${DN_PROJECT_ALIAS_PREFIX}-cds='cd ${SUPER_PROJECT_ROOT}/src'"
    echo "alias dnp-${DN_PROJECT_ALIAS_PREFIX}-cdt='cd ${SUPER_PROJECT_ROOT}/tests'"
    echo "alias dnp-${DN_PROJECT_ALIAS_PREFIX}-cda='cd ${SUPER_PROJECT_ROOT}/artifact'"
    echo "alias dnp-${DN_PROJECT_ALIAS_PREFIX}-cde='cd ${SUPER_PROJECT_ROOT}/external_data'"
    echo "#<<<<DNP ${SUPER_PROJECT_REPO_NAME:?err} aliases and env variable end"
    echo ""
  ) >>~/.bashrc

  if [ -n "$ZSH_VERSION" ]; then
    # ToDo: validate >> appending .bashrc to .zshrc should let to the user choice
    #  echo "source ~/.bashrc" >> ~/.zshrc
    source ~/.zshrc
  elif [ -n "$BASH_VERSION" ]; then
    source ~/.bashrc
  else
    n2st::print_msg_error "Unknown shell! Check with the maintainer to add it to DS"
  fi

  n2st::print_msg_done "Setup completed!

    New available alias added to ~/.bashrc:
      - dnp-${DN_PROJECT_ALIAS_PREFIX}-cd -> cd to ${SUPER_PROJECT_REPO_NAME} root
      - dnp-${DN_PROJECT_ALIAS_PREFIX}-cdd -> cd to ${SUPER_PROJECT_REPO_NAME} .dockerized_norlab_project dir
      - dnp-${DN_PROJECT_ALIAS_PREFIX}-cds -> cd to ${SUPER_PROJECT_REPO_NAME} src dir
      - dnp-${DN_PROJECT_ALIAS_PREFIX}-cdt -> cd to ${SUPER_PROJECT_REPO_NAME} tests dir
      - dnp-${DN_PROJECT_ALIAS_PREFIX}-cda -> cd to ${SUPER_PROJECT_REPO_NAME} artifact dir
      - dnp-${DN_PROJECT_ALIAS_PREFIX}-cde -> cd to ${SUPER_PROJECT_REPO_NAME} external data dir

    New available environment variable added to ~/.bashrc for convenience:
      - _DNP_${dn_project_alias_prefix_caps}_PATH=${SUPER_PROJECT_ROOT}

  "

  #  ....Teardown...................................................................................
  cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  script_path_parent="$(dirname "${script_path}")"
  source "${script_path_parent}/import_dnp_lib.bash" || exit 1
  source "${script_path_parent}/load_super_project_config.bash" || exit 1
  dnp::setup_host_for_this_super_project || exit 1
  exit $?
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
  test -n "$( declare -f dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
  test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} library load error!" ; exit 1 ; }
  test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} library load error!" ; exit 1 ; }

fi
