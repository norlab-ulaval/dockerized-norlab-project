#!/bin/bash
# install.bash

DOCUMENTATION_BUFFER_INSTALL=$(
  cat << 'EOF'
# =================================================================================================
# Install Dockerized-NorLab project application (DNA) on host.
#
# Perform the following steps:
#  1. Install software requirements if online (L4T, Ubuntu and MacOsX)
#  2. Setup cuda requirement (L4T and Ubuntu)
#  3. Set path resolution (system wide, via bashrc or none)
#  4. Print remaining setup instruction to be executed by the user (you)
#
# Usage:
#   $ bash ./install.bash [OPTIONS]
#
# Options:
#   --skip-system-wide-symlink-install  Skip creating a symlink in /usr/local/bin
#   --add-dna-path-to-bashrc            Add dna entrypoint path to ~/.bashrc instead of a system wide symlink install
#   --yes                               Bypass user interactive installation
#   --help, -h                          Show this help message
#
# =================================================================================================
EOF
)
# Set script to exit on error
set -e

# =================================================================================================
# Helper function: Create bin dna to entrypoint symlink
#
# Usage:
#   $ dna::create_bin_dna_to_entrypoint_symlink <path/to/dna/entrypoint> [<path/to/bin/dna>]
#
# Global:
#  read HOME
# =================================================================================================
function dna::create_bin_dna_to_entrypoint_symlink() {
  local dna_entrypoint_path=$1
  local bin_dna_path="${2:-/usr/local/bin/dna}"
  local show_symlink="${MSG_DIMMED_FORMAT}${bin_dna_path} -> ${dna_entrypoint_path}${MSG_END_FORMAT}"
  n2st::print_msg "Creating symlink: ${show_symlink}"
  sudo ln -sf "${dna_entrypoint_path}" "${bin_dna_path}" || return 1
  if [[ ! -h "${bin_dna_path}" ]]; then
    n2st::print_msg_error "Failed to create symlink ${show_symlink}!"
    return 1
  fi
  return 0
}

# =================================================================================================
# Helper function: Update the ~/.bashrc dna path and add it to PATH
#
# Usage:
#   $ dna::update_bashrc_dna_bin_path <path/to/dna/bin/dir>
#
# Global:
#  read HOME
# =================================================================================================
function dna::update_bashrc_dna_bin_path() {
  local dna_bin_dir="$1"
  n2st::print_msg "Updating dna entrypoint path in ~/.bashrc"
  n2st::seek_and_modify_string_in_file "export _DNA_PATH=.*" "export _DNA_PATH=\"${dna_bin_dir}\"" "${HOME}/.bashrc" || return 1
  n2st::seek_and_modify_string_in_file "export PATH=.*_DNA_PATH.*" "export PATH=\"\$PATH:\$_DNA_PATH\"" "${HOME}/.bashrc" || return 1
  return 0
}

# =================================================================================================
# Helper function: Create symlink in /usr/local/bin if requested
#
# Usage:
#   $ dna::create_entrypoint_symlink_if_requested "$option_system_wide_symlink" [yes|false] "$dna_entrypoint"
#
# =================================================================================================
function dna::create_entrypoint_symlink_if_requested() {
  local option_system_wide_symlink="$1"
  local option_yes="$2"
  local dna_entrypoint="$3"
  if [[ "${option_system_wide_symlink:?err}" == true ]]; then
    if [[ ! -d "/usr/local/bin" ]]; then
      n2st::print_msg_warning "${MSG_DIMMED_FORMAT}/usr/local/bin${MSG_END_FORMAT} directory does not exist."
      local create_and_add_to_path
      if [[ "${option_yes:?err}" == true ]]; then
        create_and_add_to_path="y"
      else
        read -r -n 1 -p "Create directory and add to PATH? [y/N] " create_and_add_to_path
      fi
      if [[ "${create_and_add_to_path}" == "y" || "${create_and_add_to_path}" == "Y" ]]; then
        sudo mkdir -p "/usr/local/bin"
        # shellcheck disable=SC2143
        if [[ -z "$( echo "${PATH}" | grep --quiet /usr/local/bin )"  ]]; then
          echo "export PATH=\"\$PATH:/usr/local/bin\"" >> "$HOME/.bashrc"
        fi
      else
        n2st::print_msg_error_and_exit "Please create /usr/local/bin directory by executing ${MSG_DIMMED_FORMAT}mkdir -p /usr/local/bin${MSG_END_FORMAT} and re-run the install script or use the ${MSG_DIMMED_FORMAT}--skip-system-wide-symlink-install${MSG_END_FORMAT} install flag."
      fi
    fi

    if [[ -L "/usr/local/bin/dna" || -f "/usr/local/bin/dna" ]]; then
      # Case symlink already exist
      if [[ "${option_yes:?err}" == false ]]; then
        n2st::print_msg_warning "${MSG_DIMMED_FORMAT}/usr/local/bin/dna${MSG_END_FORMAT} already exists"
        read -r -n 1 -p "Overwrite? [y/N] " option_overwrite
        if [[ "${option_overwrite}" != "y" && "${option_overwrite}" != "Y" ]]; then
          n2st::print_msg "Skipping symlink creation."
        else
          dna::create_bin_dna_to_entrypoint_symlink "${dna_entrypoint:?err}" || return 1
        fi
      else
        dna::create_bin_dna_to_entrypoint_symlink "${dna_entrypoint}" || return 1
      fi
    else
      dna::create_bin_dna_to_entrypoint_symlink "${dna_entrypoint}" || return 1
    fi
  fi
  return 0
}

# =================================================================================================
# Helper function: Add dna entrypoint path to ~/.bashrc if requested
#
# Usage:
#   $ dna::add_dna_entrypoint_path_to_bashrc_if_requested "$option_add_dna_path_to_bashrc" [yes|false] "$dna_bin_dir"
#
# Global:
#  read HOME
#  read BASH_SOURCE
# =================================================================================================
function dna::add_dna_entrypoint_path_to_bashrc_if_requested() {
  local option_add_dna_path_to_bashrc="$1"
  local option_yes="$2"
  local dna_bin_dir="$3"
  if [[ "${option_add_dna_path_to_bashrc}" == true ]]; then
    if [[ -f "${HOME}/.bashrc" ]]; then
      if grep --silent -E "_DNA_PATH=" "${HOME}/.bashrc" || grep --silent -E "PATH=\"\$PATH:\$_DNA_PATH" "${HOME}/.bashrc"; then
        if [[ "${option_yes}" == false ]]; then
          n2st::print_msg_warning "dna entrypoint path already exists in ~/.bashrc."
          read -r -n 1 -p "Update? [y/N] " option_update
          if [[ "${option_update}" != "y" && "${option_update}" != "Y" ]]; then
            n2st::print_msg "Skipping ~/.bashrc option_update."
          else
            dna::update_bashrc_dna_bin_path "${dna_bin_dir}"
          fi
        else
          dna::update_bashrc_dna_bin_path "${dna_bin_dir}"
        fi
      else
        n2st::print_msg "Adding dna entrypoint path to ~/.bashrc"
        {
          echo "" ;
          echo "# >>>> dockerized-norlab-project (start)" ;
          echo "export _DNA_PATH=\"${dna_bin_dir}\"" ;
          echo "export PATH=\"\$PATH:\$_DNA_PATH\"" ;
          echo "# <<<< dockerized-norlab-project (end)" ;
          echo "" ;
        } >> "${HOME}/.bashrc"
      fi
    else
      # shellcheck disable=SC2088
      n2st::print_msg_error "~/.bashrc file does not exist.\nSkipping ~/.bashrc option_update."
      return 1
    fi
  fi
  return 0
}


# =================================================================================================
# This is the main function: Install dockerized norlab project on host
#
# Usage:
#   See the script documentation at the top
#
# Global:
#  read BASH_SOURCE
# =================================================================================================
function dna::install_dockerized_norlab_project_on_host() {

  # Determine the installation directory
  dna_install_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  dna_bin_dir="${dna_install_dir}/src/bin"
  dna_entrypoint="${dna_bin_dir}/dna"

  # Source minimum required library for install purposes
  source "${dna_install_dir}/load_repo_main_dotenv.bash"
  source "${dna_install_dir}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${dna_install_dir}/src/lib/core/utils/import_dna_lib.bash"
  source "${dna_install_dir}/src/lib/core/utils/setup_host_dna_requirements.bash"

  # ....Set env variables (pre cli))...............................................................
  local option_system_wide_symlink=true
  local option_add_dna_path_to_bashrc=false
  local option_yes=false

  # ....cli........................................................................................
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --skip-system-wide-symlink-install)
        option_system_wide_symlink=false
        shift
        ;;
      --add-dna-path-to-bashrc)
        option_add_dna_path_to_bashrc=true
        option_system_wide_symlink=false
        shift
        ;;
      --yes)
        option_yes=true
        shift
        ;;
      --help | -h)
        dna::command_help_menu "${DOCUMENTATION_BUFFER_INSTALL:?err}"
        exit 0
        ;;
      *)
        dna::unknown_option_msg "bash ./install.bash" "$1"
        exit 1
        ;;
    esac
  done

  # ....Pre-condition..............................................................................
  if dna::is_online; then
    dna::check_install_darwin_package_manager "${option_yes}" || return 1
  else
    n2st::print_msg_warning "Be advised, you are currently, offline. Can't proceed with installing software requirement!"
  fi

  # ====Begin======================================================================================
  # Splash type: small, negative or big
  n2st::norlab_splash "${DNA_SPLASH_NAME_SMALL:?err}" "${DNA_GIT_REMOTE_URL}" "negative"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1:?err}"

  # ....Pre-conditions.............................................................................
  # Make the dna script is executable
  chmod +x "${dna_entrypoint}"

  # ....Setup host for this super project..........................................................
  n2st::print_msg "Setting up host..."

  if dna::is_online; then
    dna::install_dna_software_requirements || return 1
  fi
  dna::setup_cuda_requirements || return 1

  # ....Create symlink in /usr/local/bin if requested..............................................
  dna::create_entrypoint_symlink_if_requested "$option_system_wide_symlink" "$option_yes" "$dna_entrypoint" || return 1

  # ....Add dna entrypoint path to ~/.bashrc if requested..........................................
  dna::add_dna_entrypoint_path_to_bashrc_if_requested "$option_add_dna_path_to_bashrc" "$option_yes" "$dna_bin_dir" || return 1

  # ====Teardown===================================================================================
  n2st::print_msg_done "${DNA_HUMAN_NAME} has been installed successfully!"
  if [[ "${option_system_wide_symlink}" == true ]]; then
    n2st::print_msg "You can now use 'dna' command from anywhere."
    echo -e "\nRun 'dna help' for usage information."
  elif [[ "${option_add_dna_path_to_bashrc}" == true ]]; then
    n2st::print_msg "After restarting your shell or sourcing ~/.bashrc, the current user can use 'dna' command from anywhere."
    echo -e "\nRun 'dna help' for usage information."
  else
    n2st::print_msg "You can use '${dna_entrypoint}' to run DNA commands."
    echo -e "\nRun 'bash ${dna_entrypoint} help' for usage information."
  fi

  cd "${dna_install_dir}"

  if ! dna::is_online; then
    n2st::print_msg_warning "You are currently offline, software requirement install step was skiped.\nBe advise that 'docker engine', 'docker compose', 'docker buildx', 'git' and 'tree' are all hard requirement."
  fi
  if [[ $(uname -s) == "Darwin" ]]; then
    print_msg "Remaining install instructions:
1. Install 'Docker desktop' if its not already done (https://docs.docker.com/desktop/mac/install/)
2. Open Docker Desktop, go to 'Settings' and check '☑️ Start Docker Desktop when you sign in to your computer' and restart the current terminal
3. Create a multi-architecture docker builder. Execute the following comands:${MSG_DIMMED_FORMAT}
    $ docker buildx create --name local-builder-multiarch-virtual --driver docker-container --platform linux/amd64,linux/arm64 --bootstrap --use
    $ docker buildx ls
${MSG_END_FORMAT}
Stay awesome 🦾
"
  else
    print_msg "Remaining install instructions:
1. Apply Docker group change without login out: execute ${MSG_DIMMED_FORMAT}$ newgrp docker${MSG_END_FORMAT}
2. Restart the docker daemon to apply changes: execute ${MSG_DIMMED_FORMAT}$ sudo systemctl restart docker${MSG_END_FORMAT}
3. Create a multi-architecture docker builder. Execute the following comands:${MSG_DIMMED_FORMAT}
    $ docker buildx create --name local-builder-multiarch-virtual --driver docker-container --platform linux/amd64,linux/arm64 --bootstrap --use
    $ docker buildx ls
${MSG_END_FORMAT}
Stay awesome 🦾
"
  fi

  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  return 0
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  dna::install_dockerized_norlab_project_on_host "$@"
  exit $?
elif [[ -n "${BATS_TEST_FILENAME}" ]]; then
  # We're in a bats test environment, allow sourcing
  :
else
  # This script is being sourced, ie: __name__="__source__"
  dna_error_prefix="\033[1;31m[DNA error]\033[0m"
  echo -e "${dna_error_prefix} This script must be run in shell i.e.: $ bash $(basename "$0")" 1>&2
  exit 1
fi
