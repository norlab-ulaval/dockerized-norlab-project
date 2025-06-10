#!/bin/bash
# install.bash

DOCUMENTATION_BUFFER_INSTALL=$(
  cat << 'EOF'
# =================================================================================================
# Install Dockerized-NorLab-Project
#
# Usage:
#   $ bash ./install.bash [OPTIONS]
#
# Options:
#   --skip-system-wide-symlink-install  Skip creating a symlink in /usr/local/bin
#   --add-dnp-path-to-bashrc            Add dnp entrypoint path to ~/.bashrc instead of a system wide symlink install
#   --yes                               Bypass user interactive installation
#   --help, -h                          Show this help message
#
# =================================================================================================
EOF
)
# Set script to exit on error
set -e

# =================================================================================================
# Helper function: Create bin dnp to entrypoint symlink
#
# Usage:
#   $ dnp::create_bin_dnp_to_entrypoint_symlink <path/to/dnp/entrypoint> [<path/to/bin/dnp>]
#
# Global:
#  read HOME
# =================================================================================================
function dnp::create_bin_dnp_to_entrypoint_symlink() {
  local dnp_entrypoint_path=$1
  local bin_dnp_path="${2:-/usr/local/bin/dnp}"
  local show_symlink="${MSG_DIMMED_FORMAT}${bin_dnp_path} -> ${dnp_entrypoint_path}${MSG_END_FORMAT}"
  n2st::print_msg "Creating symlink: ${show_symlink}"
  sudo ln -sf "${dnp_entrypoint_path}" "${bin_dnp_path}" || return 1
  if [[ ! -h "${bin_dnp_path}" ]]; then
    n2st::print_msg_error "Failed to create symlink ${show_symlink}!"
    return 1
  fi
  return 0
}

# =================================================================================================
# Helper function: Update the ~/.bashrc dnp path and add it to PATH
#
# Usage:
#   $ dnp::update_bashrc_dnp_bin_path <path/to/dnp/bin/dir>
#
# Global:
#  read HOME
# =================================================================================================
function dnp::update_bashrc_dnp_bin_path() {
  local dnp_bin_dir="$1"
  n2st::print_msg "Updating dnp entrypoint path in ~/.bashrc"
  n2st::seek_and_modify_string_in_file "export _DNP_PATH=.*" "export _DNP_PATH=\"${dnp_bin_dir}\"" "${HOME}/.bashrc" || return 1
  n2st::seek_and_modify_string_in_file "export PATH=.*_DNP_PATH.*" "export PATH=\"\$PATH:\$_DNP_PATH\"" "${HOME}/.bashrc" || return 1
  return 0
}

# =================================================================================================
# Helper function: Create symlink in /usr/local/bin if requested
#
# Usage:
#   $ dnp::create_entrypoint_symlink_if_requested "$option_system_wide_symlink" "$option_yes" "$dnp_entrypoint"
#
# =================================================================================================
function dnp::create_entrypoint_symlink_if_requested() {
  local option_system_wide_symlink="$1"
  local option_yes="$2"
  local dnp_entrypoint="$3"
  if [[ "${option_system_wide_symlink:?err}" == true ]]; then
    if [[ ! -d "/usr/local/bin" ]]; then
      n2st::print_msg_error_and_exit "${MSG_DIMMED_FORMAT}/usr/local/bin${MSG_END_FORMAT} directory does not exist.\nPlease create it or use ${MSG_DIMMED_FORMAT}--skip-system-wide-symlink-install${MSG_END_FORMAT} option."
    fi

    if [[ -L "/usr/local/bin/dnp" || -f "/usr/local/bin/dnp" ]]; then
      # Case symlink already exist
      if [[ "${option_yes:?err}" == false ]]; then
        n2st::print_msg_warning "${MSG_DIMMED_FORMAT}/usr/local/bin/dnp${MSG_END_FORMAT} already exists"
        read -r -n 1 -p "Overwrite? [y/N] " option_overwrite
        if [[ "${option_overwrite}" != "y" && "${option_overwrite}" != "Y" ]]; then
          n2st::print_msg "Skipping symlink creation."
        else
          dnp::create_bin_dnp_to_entrypoint_symlink "${dnp_entrypoint:?err}" || return 1
        fi
      else
        dnp::create_bin_dnp_to_entrypoint_symlink "${dnp_entrypoint}" || return 1
      fi
    else
      dnp::create_bin_dnp_to_entrypoint_symlink "${dnp_entrypoint}" || return 1
    fi
  fi
  return 0
}

# =================================================================================================
# Helper function: Add dnp entrypoint path to ~/.bashrc if requested
#
# Usage:
#   $ dnp::add_dnp_entrypoint_path_to_bashrc_if_requested "$option_add_dnp_path_to_bashrc" "$option_yes" "$dnp_bin_dir"
#
# Global:
#  read HOME
#  read BASH_SOURCE
# =================================================================================================
function dnp::add_dnp_entrypoint_path_to_bashrc_if_requested() {
  local option_add_dnp_path_to_bashrc="$1"
  local option_yes="$2"
  local dnp_bin_dir="$3"
  if [[ "${option_add_dnp_path_to_bashrc}" == true ]]; then
    if [[ -f "${HOME}/.bashrc" ]]; then
      if grep --silent -E "_DNP_PATH=" "${HOME}/.bashrc" || grep --silent -E "PATH=\"\$PATH:\$_DNP_PATH" "${HOME}/.bashrc"; then
        if [[ "${option_yes}" == false ]]; then
          n2st::print_msg_warning "dnp entrypoint path already exists in ~/.bashrc."
          read -r -n 1 -p "Update? [y/N] " option_update
          if [[ "${option_update}" != "y" && "${option_update}" != "Y" ]]; then
            n2st::print_msg "Skipping ~/.bashrc option_update."
          else
            dnp::update_bashrc_dnp_bin_path "${dnp_bin_dir}"
          fi
        else
          dnp::update_bashrc_dnp_bin_path "${dnp_bin_dir}"
        fi
      else
        n2st::print_msg "Adding dnp entrypoint path to ~/.bashrc"
        {
          echo "" ;
          echo "# >>>> Dockerized-NorLab-Project (start)" ;
          echo "export _DNP_PATH=\"${dnp_bin_dir}\"" ;
          echo "export PATH=\"\$PATH:\$_DNP_PATH\"" ;
          echo "# <<<< Dockerized-NorLab-Project (end)" ;
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
function dnp::install_dockerized_norlab_project_on_host() {

  # Determine the installation directory
  dnp_install_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  dnp_bin_dir="${dnp_install_dir}/src/bin"
  dnp_entrypoint="${dnp_bin_dir}/dnp"

  # Source minimum required library for install purposes
  source "${dnp_install_dir}/load_repo_main_dotenv.bash"
  source "${dnp_install_dir}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${dnp_install_dir}/src/lib/core/utils/ui.bash"

  # ....Set env variables (pre cli))...............................................................
  local option_system_wide_symlink=true
  local option_add_dnp_path_to_bashrc=false
  local option_yes=false

  # ....cli........................................................................................
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --skip-system-wide-symlink-install)
        option_system_wide_symlink=false
        shift
        ;;
      --add-dnp-path-to-bashrc)
        option_add_dnp_path_to_bashrc=true
        option_system_wide_symlink=false
        shift
        ;;
      --yes)
        option_yes=true
        shift
        ;;
      --help | -h)
        dnp::command_help_menu "${DOCUMENTATION_BUFFER_INSTALL}"
        exit 0
        ;;
      *)
        dnp::unknown_option_msg "bash ./install.bash" "$1"
        exit 1
        ;;
    esac
  done

  # ====Begin======================================================================================
  # Splash type: small, negative or big
  n2st::norlab_splash "${DNP_GIT_NAME} (${DNP_PROMPT_NAME})" "${DNP_GIT_REMOTE_URL}" "small"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"

  # ....Pre-conditions.............................................................................
  # Make the dnp script is executable
  chmod +x "${dnp_entrypoint}"

  # ....Setup host for this super project..........................................................
  n2st::print_msg "Setting up host for Dockerized-NorLab-Project..."
  bash "${dnp_install_dir}/src/lib/core/utils/setup_host_dnp_requirements.bash" || return 1

  # ....Create symlink in /usr/local/bin if requested..............................................
  dnp::create_entrypoint_symlink_if_requested "$option_system_wide_symlink" "$option_yes" "$dnp_entrypoint" || return 1

  # ....Add dnp entrypoint path to ~/.bashrc if requested..........................................
  dnp::add_dnp_entrypoint_path_to_bashrc_if_requested "$option_add_dnp_path_to_bashrc" "$option_yes" "$dnp_bin_dir" || return 1

  # ====Teardown===================================================================================
  n2st::print_msg_done "Dockerized-NorLab-Project has been installed successfully!"
  if [[ "${option_system_wide_symlink}" == true ]]; then
    n2st::print_msg "You can now use 'dnp' command from anywhere."
    echo -e "\nRun 'dnp help' for usage information."
  elif [[ "${option_add_dnp_path_to_bashrc}" == true ]]; then
    n2st::print_msg "After restarting your shell or sourcing ~/.bashrc, the current user can use 'dnp' command from anywhere."
    echo -e "\nRun 'dnp help' for usage information."
  else
    n2st::print_msg "You can use '${dnp_entrypoint}' to run DNP commands."
    echo -e "\nRun 'bash ${dnp_entrypoint} help' for usage information."
  fi

  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  return 0
}


# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"
  dnp::install_dockerized_norlab_project_on_host "$@"
  exit $?
elif [[ -n "${BATS_TEST_FILENAME}" ]]; then
  # We're in a bats test environment, allow sourcing
  :
else
  # This script is being sourced, ie: __name__="__source__"
  dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
  echo -e "${dnp_error_prefix} This script must be run in shell i.e.: $ bash $(basename "$0")" 1>&2
  exit 1
fi
