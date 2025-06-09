#!/bin/bash
# lib/commands/init.bash

DOCUMENTATION_BUFFER_INIT=$( cat <<'EOF'
# =================================================================================================
# Initialize a new DNP project
#
# Requirements:
#   - Must be executed at target project repository root (i.e., your project top directory, aka the super project)
#   - The repository must be under version control using Git
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

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
test -n "$( declare -F dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
test -n "$( declare -F n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }
test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
function dnp::get_acronym() {
    # Find the acronym of a dash or underscrore sepatared multi-word name.
    local super_project_name="$1"
    echo "$super_project_name" | grep -o -P '(^|\-|\_)[a-zA-Z0-9]' | tr -d '-' | tr -d '_' | tr -d '\n' | tr '[:upper:]' '[:lower:]'
    return 0
}

function dnp::get_super_project_acronym() {
    # Find the acronym of a dash or underscrore sepatared multi-word name.
    # Return the three first letter if the super_project_name is not a multi-word.
    local super_project_name="$1"
    local acronym

    # ....Get acronym assuming the name is multi-word..............................................
    acronym=$(dnp::get_acronym "$super_project_name")

    # ....Pick the first three character otherwise.................................................
    if [ ${#acronym} -lt 2 ]; then
        if [ ${#super_project_name} -ge 3 ]; then
            echo "${super_project_name:0:3}" | tr '[:upper:]' '[:lower:]'
        fi
    else
        # The name is multi-word, so output the acronym
        echo "$acronym"
    fi
    return 0
}


function dnp::init_command() {
    local super_project_root
    super_project_root=$(pwd)

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_INIT}"
                exit 0
                ;;
            *)
                dnp::unknown_subcommand_msg "init" "$*"
                return 1
                ;;
        esac
    done

    # ....Pre-condition............................................................................
    # Validate that we're at super project repository root
    if [[ ! -d ".git" ]]; then
        n2st::print_msg_error_and_exit "Cwd is not at repository root.
    Please run this command from the root of your project repository.
    Current workin directory: $(pwd)"
    fi

    local project_git_remote_url
    local super_project_name
    local super_project_user
    project_git_remote_url="$( git remote get-url origin )"
    super_project_name="$( basename "${project_git_remote_url}" .git )}"
    super_project_user="$(id -un)"

    # Check if .dockerized_norlab_project already exists
    if [[ -d ".dockerized_norlab_project" ]]; then
        n2st::print_msg_error_and_exit "${MSG_DIMMED_FORMAT}.dockerized_norlab_project${MSG_END_FORMAT} directory already exists.\nThis project is already DNP initialized."
    fi

    # ====Begin====================================================================================
    n2st::print_msg "Initializing DNP project: ${super_project_name} in ${super_project_root}"

    # Create the .dockerized_norlab_project directory
    mkdir -p .dockerized_norlab_project

    # Copy template files
    cp -r "${DNP_LIB_PATH}/template/.dockerized_norlab_project/"* .dockerized_norlab_project/

    cd "${super_project_root}/.dockerized_norlab_project/" || exit 1

    # Rename the super project DNP meta .env file
    cp ".env.PLACEHOLDER_SUPER_PROJECT_NAME" ".env.${super_project_name}"

    # Replace placeholders in the .env.dnp file
    cd "${super_project_root}/.dockerized_norlab_project/configuration/" || exit 1
    local super_project_acronym
    super_project_acronym="$(dnp::get_super_project_acronym "${super_project_name}")"
    n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_PROJECT_GIT_REMOTE_URL" "${project_git_remote_url}" ".env.dnp"
    n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_CONTAINER_NAME" "IamDNP_${super_project_acronym}" ".env.dnp"
    n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_PROJECT_ALIAS_PREFIX" "${super_project_acronym}" ".env.dnp"

    # Replace placeholders in the DNP readme file
    cd "${super_project_root}/.dockerized_norlab_project/" || exit 1
    n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_CONTAINER_NAME" "IamDNP_${super_project_acronym}" "README.md"
    n2st::seek_and_modify_string_in_file "PLACEHOLDER_SUPER_PROJECT_NAME" "${super_project_name}" "README.md"
    n2st::seek_and_modify_string_in_file "PLACEHOLDER_SUPER_PROJECT_USER" "${super_project_user}" "README.md"

    # ....Create root repository required directories..............................................
    cd "${super_project_root}" || exit 1

    mkdir -p artifact
    mkdir -p external_data
    mkdir -p src/launcher
    mkdir -p src/tools
    mkdir -p tests

    # ....Create root README.md files if it does't exist...........................................
    cd "${super_project_root}" || exit 1

    if [[ ! -f "README.md" ]]; then
        cat > "README.md" << EOF
# ${super_project_name}

This project is initialized with [Dockerized-NorLab-Project](https://github.com/norlab-ulaval/dockerized-norlab-project.git).
See '.dockerized_norlab_project/README.md' for usage details.

EOF
    fi

    # ....Setup ignore files.......................................................................
    cd "${super_project_root}" || exit 1

    if [[ ! -f ".gitignore" ]]; then
        # Case: file does not exist => copy template
        cp "${DNP_LIB_PATH}/template/.gitignore" .gitignore
    else
        # Case: file exist => append required .gitignore entries
        cat >> ".gitignore" << EOF

# ====Dockerized-NorLab(required)==================================================================
**/.dockerized_norlab_project/dn_container_env_variable/
**/.dockerized_norlab_project/configuration/.env.local

# ====Dockerized-NorLab(recommended)===============================================================
**/external_data/
**/artifact/
!**/external_data/README.md
!**/artifact/README.md
!**/artifact/optuna_storage/README.md
**/slurm_jobs_logs/*.log

EOF
    fi

    if [[ ! -f ".dockerignore" ]]; then
        # Case: file does not exist => copy template
        cp "${DNP_LIB_PATH}/template/.dockerignore" .dockerignore
    else
        # Case: file exist => append required .gitignore entries
      cat >> ".dockerignore" << EOF

# ====Dockerized-NorLab(required)==================================================================
!**/.dockerized_norlab_project/
!**/version.txt
!**/.git

# ====Dockerized-NorLab(recommended)===============================================================
**/external_data/
**/artifact/
**/slurm_jobs_logs/*.log

EOF
    fi

    # ....Validate init procedure..................................................................
    cd "${super_project_root}" || exit 1

    source "${DNP_LIB_PATH}/core/utils/super_project_dnp_sanity_check.bash" || return 1

    # ====Teardown=================================================================================
    n2st::print_msg "DNP project initialized successfully.
You can now use ${MSG_DIMMED_FORMAT}dnp${MSG_END_FORMAT} to manage your project.
To get started:
  1. Execute ${MSG_DIMMED_FORMAT}dnp help${MSG_END_FORMAT} to see available command
  2. Read instruction in '.dockerized_norlab_project/README.md'
  3. Check documentation at https://github.com/norlab-ulaval/dockerized-norlab-project.git.
  4. Stay awesome
"
    cd "$super_project_root" || exit 1
    return 0
}
