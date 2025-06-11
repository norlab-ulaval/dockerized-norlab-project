#!/bin/bash
# =================================================================================================
# Setup host computer (i.e, workstation, embed computer, server) for using this
# Dockerized-NorLab-Project.
#
# Usage:
#   $ bash setup_host_dnp_requirements.bash
#   or
#   $ source setup_host_dnp_requirements.bash && dnp::setup_host_dnp_requirements
#
# Global:
#  read DNP_ROOT
#  read PATH
#  read LD_LIBRARY_PATH
#
# =================================================================================================

function dnp::setup_host_dnp_requirements() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Install docker requirements................................................................
  cd "${DNP_ROOT}/utilities/norlab-shell-script-tools/src/utility_scripts" || return 1
  bash install_docker_tools.bash || return 1

  cd "$tmp_cwd" || return 1

  if [ -n "$ZSH_VERSION" ]; then
    source "$HOME/.zshrc"
  elif [ -n "$BASH_VERSION" ]; then
    source "$HOME/.bashrc"
  else
    n2st::print_msg_error "Unknown shell! Check with the maintainer to add its support to DNP."
  fi


  # ...CUDA toolkit path...........................................................................
  # ref dusty_nv comment at https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068
  if [[ $(uname -s) == "Darwin" ]]; then
    n2st::print_msg_warning "CUDA is not supported yet on Apple M1 computer. Skipping cuda configuration."
  else
    if ! command -V nvcc -V &>/dev/null; then
      # nvcc command not working
      n2st::print_msg_warning "Fixing CUDA path for nvcc"

      (
        echo ""
        echo "# >>>> Dockerized-NorLab-Project CUDA (start)"
        echo "# CUDA toolkit related"
        echo "# ref dusty_nv comment at"
        echo "#    https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068"
        echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}"
        echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
        echo "# <<<< Dockerized-NorLab-Project CUDA (end)"
        echo ""
      ) >> "$HOME/.bashrc"

      source "$HOME/.bashrc"
      n2st::print_msg_done "nvcc CUDA path hack completed. The following lines where added to ~/.bashrc
      ${MSG_DIMMED_FORMAT}
      # CUDA toolkit related
      # ref dusty_nv comment at
      #    https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068
      export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
      export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}
      ${MSG_END_FORMAT}"
    fi

    if command -V nvcc -V &>/dev/null; then
      if [[ $(nvcc -V | grep 'nvcc: NVIDIA (R) Cuda compiler driver') == "nvcc: NVIDIA (R) Cuda compiler driver" ]]; then
        n2st::print_msg_done "nvcc installed properly"
        nvcc -V
      else
        n2st::print_msg_warning "Check your nvcc installation. It's stil NOT installed properly!"
      fi
    else
      n2st::print_msg_warning "Check your nvcc installation. It's NOT installed!"
    fi
  fi

  if [ -n "$ZSH_VERSION" ]; then
    n2st::print_msg_warning "You are currently in a zsh shell. We made modification to the .bashrc.\nWe recommand you add 'source \$HOME/.bashrc' to your '.zshrc' file or copy those lines to it."
    source "$HOME/.zshrc"
  elif [ -n "$BASH_VERSION" ]; then
    source "$HOME/.bashrc"
  else
    n2st::print_msg_error "Unknown shell! Check with the maintainer to add its support to DNP."
  fi


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
  dnp::setup_host_dnp_requirements || exit 1
  exit $?
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
  test -n "$( declare -F dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
  test -n "$( declare -F n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
  test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }
  test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }

fi
