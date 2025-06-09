#!/bin/bash
# =================================================================================================
# Setup host computer (i.e, workstation, embed computer, server) for using this
# Dockerized-NorLab-Project super project.
#
# Usage:
#   $ source setup_host_for_running_this_super_project.bash
#
# Global:
#  read SUPER_PROJECT_ROOT
#  read SUPER_PROJECT_REPO_NAME
#  read DN_PROJECT_ALIAS_PREFIX
#  read PATH
#  read LD_LIBRARY_PATH
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

  if [[ -z "${DNP_ROOT}" ]]; then
    echo -e "\033[1;31m[DNP error]\033[0m DNP libs are not loaded, run import_dnp_lib.bash first!" 1>&2
    exit 1
  fi
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

  # ...CUDA toolkit path...........................................................................
  # ref dusty_nv comment at https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068
  if [[ $(uname -s) == "Darwin" ]]; then
    n2st::print_msg_warning "CUDA is not supported yet on Apple M1 computer"
  else
    if ! command -v nvcc -V &>/dev/null; then
      # nvcc command not working
      n2st::print_msg_warning "Fixing CUDA path for nvcc"

      (
        echo ""
        echo "# CUDA toolkit related"
        echo "# ref dusty_nv comment at"
        echo "#    https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068"
        echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}"
        echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
        echo ""
      ) >>~/.bashrc

      source ~/.bashrc
      n2st::print_msg_done "nvcc CUDA path hack completed. The following lines where added to ~/.bashrc
      ${MSG_DIMMED_FORMAT}
      # CUDA toolkit related
      # ref dusty_nv comment at
      #    https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068
      export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
      export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
      ${MSG_END_FORMAT}"
    fi

    if [[ $(nvcc -V | grep 'nvcc: NVIDIA (R) Cuda compiler driver') == "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
      n2st::print_msg_done "nvcc installed properly"
      nvcc -V
    else
      n2st::print_msg_error " Check your nvcc installation. It's stil NOT installed properly!"
    fi
  fi

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
  script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  script_path_parent="$(dirname "${script_path}")"
  if [[ -z $( declare -F dnp::import_lib_and_dependencies ) ]]; then
    source "${script_path_parent}/../utils/import_dnp_lib.bash" || exit 1
  fi
  if [[ -z ${SUPER_PROJECT_ROOT} ]]; then
    source "${script_path_parent}/../utils/load_super_project_config.bash" || exit 1
  fi
  dnp::setup_host_for_this_super_project || exit 1
else
  # This script is being sourced, ie: __name__="__source__"
  :
fi
