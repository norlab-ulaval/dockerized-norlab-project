#!/bin/bash
# lib/commands/init.bash

DOCUMENTATION_BUFFER_INIT=$( cat <<'EOF'
# =================================================================================================
# Initialize a new DNP project
#
# Note: execuet at target repository root (i.e., your project top directory)
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
function dnp::init_command() {
    local project_name=""

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dnp::command_help_menu "${DOCUMENTATION_BUFFER_INIT}"
                exit 0
                ;;
            *)
                dnp::unknown_subcommand_msg "init" "${sub_the_command}"
                return 1
                ;;
        esac
    done

    # ....Pre-condition............................................................................
    # Validate that we're at target repository root
    if [[ ! -d ".git" ]]; then
        n2st::print_msg_error "Not at repository root.\nPlease run this command from the root of your project repository."
        exit 1
    fi

    local project_git_remote_url
    project_git_remote_url="$( git remote get-url origin )"
    project_name="$( basename "${project_git_remote_url}" .git )}"

    # Check if .dockerized_norlab_project already exists
    if [[ -d ".dockerized_norlab_project" ]]; then
        n2st::print_msg_error "${MSG_DIMMED_FORMAT}.dockerized_norlab_project${MSG_END_FORMAT} directory already exists.\nThis project is already DNP initialized." >&2
        exit 1
    fi

    # ====Begin====================================================================================
    n2st::print_msg "Initializing DNP project: ${project_name}"

    # Create the .dockerized_norlab_project directory
    mkdir -p .dockerized_norlab_project

    # Copy template files
    cp -r "${DNP_LIB_PATH}/template/.dockerized_norlab_project/"* .dockerized_norlab_project/

    # Create the .env.${project_name} file
    cp "${DNP_LIB_PATH}/template/.dockerized_norlab_project/.env.SUPER-PROJECT-NAME-PLACEHOLDER" ".env.${project_name}"


    # (Priority) ToDo: .env placeholder substitution (ref task NMO-656)
#    # Replace placeholders in the .env file
#    test -d configuration && test -f configuration/.env || exit 1
#    n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_CONTAINER_NAME" "TODO" "configuration/.env"
#    n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_PROJECT_ALIAS_PREFIX" "TODO" "configuration/.env"

    # ....Create root repository required directories..............................................
    mkdir -p artifact
    mkdir -p external_data
    mkdir -p src/launcher
    mkdir -p src/tools
    mkdir -p tests

    # ....Create main README.md files if it does't exist...........................................
    if [[ ! -f "README.md" ]]; then
        cat > "README.md" << EOF
# ${project_name}

This project is initialized with [Dockerized-NorLab-Project](https://github.com/norlab-ulaval/dockerized-norlab-project.git).
See `.dockerized_norlab_project/README.md` for usage details.

EOF
    fi

    # ....Setup ignore files.......................................................................
    if [[ ! -f ".gitignore" ]]; then
        cp "${DNP_LIB_PATH}/template/.gitignore" .gitignore
    else
      # (CRITICAL) ToDo: prepend required .gitignore entries
      :
    fi

    if [[ ! -f ".dockerignore" ]]; then
        cp "${DNP_LIB_PATH}/template/.dockerignore" .dockerignore
    else
      # (CRITICAL) ToDo: prepend required .dockerignore entries
      :
    fi

    # ....Validate init procedure..................................................................
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
    return 0
}
