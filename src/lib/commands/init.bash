#!/bin/bash
# lib/commands/init.bash

DOCUMENTATION_BUFFER_INIT=$( cat <<'EOF'
# =================================================================================================
# Initialize a new DNP project
#
# Usage:
#   $ dnp init [--help]
#
# Arguments:
#   --help, -h             Show this help message
#
# =================================================================================================
EOF
)

## Source the import_dnp_lib.bash script to get access to n2st::seek_and_modify_string_in_file
#source "${DNP_LIB_PATH:?err}/core/utils/import_dnp_lib.bash"

test -d "${DNP_ROOT:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo "The DNP lib load error!" ; exit 1 ; }

function dnp::init() {
    local project_name=""

    # Parse options
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::init_help
                exit 0
                ;;
            *)
                echo "Error: Too many arguments." >&2
                dnp::init_help
                exit 1
                ;;
        esac
    done


    # Validate that we're in a git repository root
    if [[ ! -d ".git" ]]; then
        echo "Error: Not in a git repository root directory." >&2
        echo "Please run this command from the root of your project repository." >&2
        exit 1
    fi

    local project_git_remote_url
    project_git_remote_url="$( git remote get-url origin )"
    project_name="$( basename "${project_git_remote_url}" .git )}"

    # Check if .dockerized_norlab_project already exists
    if [[ -d ".dockerized_norlab_project" ]]; then
        echo "Error: .dockerized_norlab_project directory already exists." >&2
        echo "This project appears to be already initialized." >&2
        exit 1
    fi

    echo "Initializing DNP project: ${project_name}"

    # Create the .dockerized_norlab_project directory
    mkdir -p .dockerized_norlab_project

    # Copy template files
    cp -r "${DNP_LIB_PATH}/template/.dockerized_norlab_project/"* .dockerized_norlab_project/

    # Create the .env.${project_name} file
    cp "${DNP_LIB_PATH}/template/.dockerized_norlab_project/.env.SUPER-PROJECT-NAME-PLACEHOLDER" ".env.${project_name}"

    test -d configuration
    test -f configuration/.env

    # (Priority) ToDo: .env placeholder substitution (ref task NMO-656)
#    # Replace placeholders in the .env file
#    n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_CONTAINER_NAME" "TODO" "configuration/.env"
#    n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_PROJECT_ALIAS_PREFIX" "TODO" "configuration/.env"

    # Create other required directories
    mkdir -p artifact
    mkdir -p external_data
    mkdir -p src/launcher
    mkdir -p src/tools
    mkdir -p tests

    # Create README.md files if they don't exist
    if [[ ! -f "README.md" ]]; then
        echo "# ${project_name}" > README.md
        echo "" >> README.md
        echo "A project using Dockerized-NorLab-Project." >> README.md
    fi

    if [[ ! -f "src/README.md" ]]; then
        echo "# ${project_name} Source Code" > src/README.md
        echo "" >> src/README.md
        echo "This directory contains the source code for ${project_name}." >> src/README.md
    fi

    # Copy .gitignore and .dockerignore if they don't exist
    if [[ ! -f ".gitignore" ]]; then
        cp "${DNP_LIB_PATH}/template/.gitignore" .gitignore
    fi

    if [[ ! -f ".dockerignore" ]]; then
        cp "${DNP_LIB_PATH}/template/.dockerignore" .dockerignore
    fi

    # Validate the setup
    source "${DNP_LIB_PATH}/core/utils/super_project_dnp_sanity_check.bash"

    echo "DNP project initialized successfully."
    echo "You can now use 'dnp build', 'dnp up', etc. to manage your project."
    return 0
}

function dnp::init_help() {
    echo -e "${MSG_DIMMED_FORMAT}"
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "$0 init --help"
    # Strip shell comment char `#` and both lines
    echo -e "${DOCUMENTATION_BUFFER_INIT}" | sed 's/\# ====.*//' | sed 's/^\#//'
    n2st::draw_horizontal_line_across_the_terminal_window "="
    echo -e "${MSG_END_FORMAT}"
}
