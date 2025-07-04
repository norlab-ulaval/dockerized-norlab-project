#!/bin/bash
# =================================================================================================
# Setup Dockerized-NorLab project application requirement on host computer.
#
# Usage:
#   $ bash setup_host_dna_requirements.bash
#   or
#   $ source setup_host_dna_requirements.bash && dna::setup_host_dna_requirements
#   or
#   $ source setup_host_dna_requirements.bash \
#         && dna::check_install_darwin_package_manager [yes] \
#         && dna::install_dna_software_requirements \
#         && dna::setup_cuda_requirements
#
# Global:
#  read DNA_ROOT
#  read PATH
#  read LD_LIBRARY_PATH
#
# =================================================================================================


# =================================================================================================
# Check if Brew or MacPort is present on host. Optionally install Brew or exit.
# Skip on non-MacOsX operating system
#
# Pre-condition:
# - Assume host is online
#
# Usage:
#   $ dna::check_install_darwin_package_manager [yes]
#
# Argument:
#   [yes]     Assume going forward with installing Brew if not installed
#
# =================================================================================================
function dna::check_install_darwin_package_manager() {
  local option_yes="${1:-false}"

  if [[ $(uname) == "Darwin" ]]; then
    if command -v brew >/dev/null 2>&1; then
        n2st::print_msg "Using Homebrew for package management"
    elif command -v port >/dev/null 2>&1; then
        n2st::print_msg "Using MacPorts for package management"
    elif [[ "${option_yes}" == yes ]]; then
      # Installing Homebreww
      /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
    else
      n2st::print_msg_warning "Neither Homebrew nor MacPorts is installed. Would you like to instal Homebreww now?"
      unset option_update
      read -r -n 1 -p "Install Homebrew? [y/N] " option_update
      if [[ "${option_update}" == "y" || "${option_update}" == "Y" ]]; then
        # Installing Homebreww
        /bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
      else
        n2st::print_msg_error "Please install Homebrew or MacPorts to continue.
  https://brew.sh
  https://ports.macports.org"
        exit 1
      fi
    fi

    if [[ "${option_yes}" == yes ]] || [[ "${option_update}" == "y" || "${option_update}" == "Y" ]]; then
      # Post install step
      if [[ "$( which bash )"  =~ .*"/bash" ]]; then
        if [[ ! -f "$HOME/.bashrc" ]]; then
          touch "$HOME/.bashrc"
        fi
        cat >> "$HOME/.bashrc" << EOF

eval "$(/opt/homebrew/bin/brew shellenv)"

EOF
        source "$HOME/.bashrc"
      fi

      if [[ "$( which zsh )" =~ .*"/zsh" ]]; then
        if [[ ! -f "$HOME/.zshrc" ]]; then
          touch "$HOME/.zshrc"
        fi
        cat >> "$HOME/.zshrc" << EOF

source $HOME/.bashrc

EOF
        source "$HOME/.zshrc"
      fi

      if [[ ! "$( which zsh )" =~ .*"/zsh" ]] && [[ ! "$( which bash )" =~ .*"/bash" ]]; then
        n2st::print_msg_warning "Unknown shell ${SHELL}! Check with the maintainer to add its support to DNA."
        n2st::print_msg_warning "Brew post-installation step. You need to update your shell’s config file (example ~/.bashrc or ~/.zshrc) to include this:\n
   ${MSG_WARNING_FORMAT}eval \"$(/opt/homebrew/bin/brew shellenv)\"${MSG_END_FORMAT}\n"
      fi
    fi
  fi

  return 0
}

# =================================================================================================
# Install DNA software requirement. Support Linux based and MacOsX operating system.
#
# Pre-condition:
# - Assume host is online
# - Assume Brew or MacPort are installed on Darwin host
#
# Usage:
#   $ dna::install_dna_software_requirements
#
# =================================================================================================
function dna::install_dna_software_requirements() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ....Install docker requirements................................................................
  if [[ $(uname -s) == "Darwin" ]]; then
    # Docker engine, compose and buildx are included in Docker Desktop
    if command -v docker >/dev/null 2>&1; then
      n2st::print_msg "'Docker Desktop' is installed."
    else
      n2st::print_msg_warning "'Docker Desktop' is not installed."
    fi
  else
      cd "${DNA_ROOT}/utilities/norlab-shell-script-tools/src/utility_scripts" || return 1
      bash install_docker_tools.bash || return 1
  fi

  if [[ -n "${ZSH_VERSION}" ]] && [[ -f "$HOME/.zshrc" ]]; then
    source "$HOME/.zshrc"
  elif [[ -n "${BASH_VERSION}" ]] && [[ -f "$HOME/.bashrc" ]]; then
    source "$HOME/.bashrc"
  else
    n2st::print_msg_warning "Unknown shell ${SHELL} ! Check with the maintainer to add its support to DNA."
  fi

  cd "$tmp_cwd" || return 1

  # ....Install other tools........................................................................
  if [[ $(uname) == "Darwin" ]]; then
    if command -v brew >/dev/null 2>&1; then
        n2st::print_msg "Using Homebrew for package management"
        brew update \
          && brew install git \
          && brew install tree \
          && brew install rsync \
          || return 1
    elif command -v port >/dev/null 2>&1; then
        n2st::print_msg "Using MacPorts for package management"
        sudo port install git \
          && sudo port install tree \
          && sudo port install rsync \
          || exit 1
    else
        n2st::print_msg_error "Neither Homebrew nor MacPorts is installed" && return 1
    fi
  else
    sudo apt-get update &&
      sudo apt-get install --assume-yes \
        git \
        tree \
        rsync \
      || return 1
  fi


  #  ....Teardown..................................................................................
  cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# =================================================================================================
# Setup cuda related requirement.
# Will issue a warning and skip on MacOsX.
#
# Usage:
#   $ dna::setup_cuda_requirements
#
# =================================================================================================
function dna::setup_cuda_requirements() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  # ...CUDA toolkit path...........................................................................
  # ref dusty_nv comment at https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068
  if [[ $(uname -s) == "Darwin" ]]; then
    n2st::print_msg_warning "CUDA is not supported yet on Apple M1 computer. Skipping cuda configuration."
  else
    if ! command -V nvcc -V &>/dev/null; then
      # nvcc command not working
      n2st::print_msg_warning "Fixing CUDA path for nvcc"

      if [[ ! -f "$HOME/.bashrc" ]]; then
        touch "$HOME/.bashrc"
      fi

      (
        echo ""
        echo "# >>>> dockerized-norlab-project CUDA (start)"
        echo "# CUDA toolkit related"
        echo "# ref dusty_nv comment at"
        echo "#    https://forums.developer.nvidia.com/t/cuda-nvcc-not-found/118068"
        echo "export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}"
        echo "export LD_LIBRARY_PATH=/usr/local/cuda/lib64${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}"
        echo "# <<<< dockerized-norlab-project CUDA (end)"
        echo ""
      ) >> "$HOME/.bashrc"

      if [[ -n "${ZSH_VERSION}" ]] && [[ -f "$HOME/.zshrc" ]]; then
        n2st::print_msg_warning "You are currently in a zsh shell. We made modification to the .bashrc.\nWe recommand you add 'source \$HOME/.bashrc' to your '.zshrc' file or copy those lines to it."
        source "$HOME/.zshrc"
      elif [[ -n "${BASH_VERSION}" ]] && [[ -f "$HOME/.bashrc" ]]; then
        source "$HOME/.bashrc"
      else
        n2st::print_msg_warning "Unknown shell ${SHELL} ! Check with the maintainer to add its support to DNA."
      fi


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

  #  ....Teardown..................................................................................
  cd "${tmp_cwd}" || { n2st::print_msg_error "Return to original dir error" 1>&2 && return 1; }
  return 0
}

# =================================================================================================
# Convenience function which perform all DNA requirement setup procedure
#
# Usage:
#   $ dna::setup_host_dna_requirements
#
# =================================================================================================
function dna::setup_host_dna_requirements() {
  local option_yes="${1:-false}"
  dna::check_install_darwin_package_manager "${option_yes}" || return 1
  dna::install_dna_software_requirements || return 1
  dna::setup_cuda_requirements || return 1
  return 0
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
  script_path_parent="$(dirname "${script_path}")"
  source "${script_path_parent}/import_dna_lib.bash" || exit 1
  dna::setup_host_dna_requirements "$@" || exit 1
  exit $?
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dna_error_prefix="\033[1;31m[DNA error]\033[0m"
  test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
  test -d "${DNA_ROOT:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }
  test -d "${DNA_LIB_PATH:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

fi
