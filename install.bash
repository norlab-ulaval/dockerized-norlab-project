#!/bin/bash
# install.bash

DOCUMENTATION_BUFFER_INSTALL=$( cat <<'EOF'
# =================================================================================================
# Install Dockerized-NorLab-Project
#
# Usage:
#   $ ./install.bash [OPTIONS]
#
# Options:
#   --skip-system-wide-symlink-install  Skip creating a symlink in /usr/local/bin
#   --add-dnp-path-to-bashrc            Add DNP_PATH to ~/.bashrc
#   --yes                               Bypass user interactive installation
#   --help, -h                          Show this help message
#
# =================================================================================================
EOF
)

# Set script to exit on error
set -e

# Determine the installation directory
DNP_INSTALL_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DNP_BIN_DIR="${DNP_INSTALL_DIR}/src/bin"
DNP_SCRIPT="${DNP_BIN_DIR}/dnp"


# Source the N2ST library to get access to n2st::seek_and_modify_string_in_file
source "${DNP_INSTALL_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"

# Show help message
function show_help() {
  # (NICE TO HAVE) ToDo: refactor as a n2st fct (ref NMO-583)
  echo -e "${MSG_DIMMED_FORMAT}"
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "$0 --help"
  # Strip shell comment char `#` and both lines
  echo -e "${DOCUMENTATION_BUFFER_INSTALL}" | sed 's/\# ====.*//' | sed 's/^\#//'
  n2st::draw_horizontal_line_across_the_terminal_window "="
  echo -e "${MSG_END_FORMAT}"
}

# Default options
SYSTEM_WIDE_SYMLINK=true
ADD_DNP_PATH_TO_BASHRC=false
YES=false

# Parse options
while [[ $# -gt 0 ]]; do
    case "$1" in
        --skip-system-wide-symlink-install)
            SYSTEM_WIDE_SYMLINK=false
            shift
            ;;
        --add-dnp-path-to-bashrc)
            ADD_DNP_PATH_TO_BASHRC=true
            shift
            ;;
        --yes)
            YES=true
            shift
            ;;
        --help|-h)
            show_help
            exit 0
            ;;
        *)
            echo "Error: Unknown option: $1" >&2
            echo "Run './install.bash --help' for usage information." >&2
            exit 1
            ;;
    esac
done


# Make sure the dnp script is executable
chmod +x "${DNP_SCRIPT}"

# Create symlink in /usr/local/bin if requested
if [[ "${SYSTEM_WIDE_SYMLINK}" == true ]]; then
    if [[ ! -d "/usr/local/bin" ]]; then
        echo "Error: /usr/local/bin directory does not exist." >&2
        echo "Please create it or use --skip-system-wide-symlink-install option." >&2
        exit 1
    fi

    if [[ -L "/usr/local/bin/dnp" || -f "/usr/local/bin/dnp" ]]; then
        if [[ "${YES}" == false ]]; then
            read -p "Warning: /usr/local/bin/dnp already exists. Overwrite? [y/N] " OVERWRITE
            if [[ "${OVERWRITE}" != "y" && "${OVERWRITE}" != "Y" ]]; then
                echo "Skipping symlink creation."
            else
                echo "Creating symlink: /usr/local/bin/dnp -> ${DNP_SCRIPT}"
                sudo ln -sf "${DNP_SCRIPT}" /usr/local/bin/dnp
            fi
        else
            echo "Creating symlink: /usr/local/bin/dnp -> ${DNP_SCRIPT}"
            sudo ln -sf "${DNP_SCRIPT}" /usr/local/bin/dnp
        fi
    else
        echo "Creating symlink: /usr/local/bin/dnp -> ${DNP_SCRIPT}"
        sudo ln -sf "${DNP_SCRIPT}" /usr/local/bin/dnp
    fi
fi

# Add DNP_PATH to ~/.bashrc if requested
if [[ "${ADD_DNP_PATH_TO_BASHRC}" == true ]]; then
    if [[ -f "${HOME}/.bashrc" ]]; then
        if grep -q "DNP_PATH=" "${HOME}/.bashrc"; then
            if [[ "${YES}" == false ]]; then
                read -p "Warning: DNP_PATH already exists in ~/.bashrc. Update? [y/N] " UPDATE
                if [[ "${UPDATE}" != "y" && "${UPDATE}" != "Y" ]]; then
                    echo "Skipping ~/.bashrc update."
                else
                    echo "Updating DNP_PATH in ~/.bashrc"
                    # Replace lines containing DNP_PATH with empty string (effectively deleting them)
                    n2st::seek_and_modify_string_in_file "export DNP_PATH=.*" "" "${HOME}/.bashrc"
                    n2st::seek_and_modify_string_in_file "export PATH=.*DNP_PATH.*" "" "${HOME}/.bashrc"
                    echo "export DNP_PATH=\"${DNP_BIN_DIR}\"" >> "${HOME}/.bashrc"
                    echo "export PATH=\"\${PATH}:\${DNP_PATH}\"" >> "${HOME}/.bashrc"
                fi
            else
                echo "Updating DNP_PATH in ~/.bashrc"
                # Replace lines containing DNP_PATH with empty string (effectively deleting them)
                n2st::seek_and_modify_string_in_file "export DNP_PATH=.*" "" "${HOME}/.bashrc"
                n2st::seek_and_modify_string_in_file "export PATH=.*DNP_PATH.*" "" "${HOME}/.bashrc"
                echo "export DNP_PATH=\"${DNP_BIN_DIR}\"" >> "${HOME}/.bashrc"
                echo "export PATH=\"\${PATH}:\${DNP_PATH}\"" >> "${HOME}/.bashrc"
            fi
        else
            echo "Adding DNP_PATH to ~/.bashrc"
            echo "" >> "${HOME}/.bashrc"
            echo "# Dockerized-NorLab-Project" >> "${HOME}/.bashrc"
            echo "export DNP_PATH=\"${DNP_BIN_DIR}\"" >> "${HOME}/.bashrc"
            echo "export PATH=\"\${PATH}:\${DNP_PATH}\"" >> "${HOME}/.bashrc"
        fi
    else
        echo "Error: ~/.bashrc file does not exist." >&2
        echo "Skipping ~/.bashrc update." >&2
    fi
fi

# Setup host for this super project
echo "Setting up host for Dockerized-NorLab-Project..."
source "${DNP_INSTALL_DIR}/src/lib/core/utils/setup_host_for_running_this_super_project.bash"

# Validate the installation
echo "Validating installation..."
source "${DNP_INSTALL_DIR}/src/lib/core/utils/super_project_dnp_sanity_check.bash"

echo ""
echo "Dockerized-NorLab-Project has been installed successfully!"
echo ""
if [[ "${SYSTEM_WIDE_SYMLINK}" == true ]]; then
    echo "You can now use 'dnp' command from anywhere."
else
    echo "You can use '${DNP_SCRIPT}' to run DNP commands."
    if [[ "${ADD_DNP_PATH_TO_BASHRC}" == true ]]; then
        echo "After restarting your shell or sourcing ~/.bashrc, you can use 'dnp' command from anywhere."
    fi
fi
echo ""
echo "Run 'dnp help' for usage information."
