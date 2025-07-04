#!/bin/bash
# lib/commands/init.bash

DOCUMENTATION_BUFFER_INIT=$( cat <<'EOF'
# =================================================================================================
# Initialize a new DNA project
#
# Requirements:
#   - Must be executed at target project repository root (i.e., your project top directory, aka the super project)
#   - The repository must be under version control using Git
#
# Usage:
#   $ dna init [--help]
#
# Arguments:
#   --help, -h             Show this help message
#
# =================================================================================================
EOF
)

# ::::Pre-condition::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
dna_error_prefix="\033[1;31m[DNA error]\033[0m"
test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
test -d "${DNA_ROOT:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }
test -d "${DNA_LIB_PATH:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

# ::::Command functions::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

function dna::get_acronym() {
    # Find the acronym of a dash or underscrore sepatared multi-word name.
    local super_project_name="$1"
    echo "$super_project_name" | grep -o -P '(^|\-|\_)[a-zA-Z0-9]' | tr -d '-' | tr -d '_' | tr -d '\n' | tr '[:upper:]' '[:lower:]'
    return 0
}

function dna::get_super_project_acronym() {
    # Find the acronym of a dash or underscrore sepatared multi-word name.
    # Return the three first letter if the super_project_name is not a multi-word.
    local super_project_name="$1"
    local acronym

    # ....Get acronym assuming the name is multi-word..............................................
    acronym=$(dna::get_acronym "$super_project_name")

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

function dna::dimmed_rsync() {
    DIM=$(tput dim)
    RESET=$(tput sgr0)
    n2st::draw_horizontal_line_across_the_terminal_window "─" "${DIM}"
    rsync "$@" | sed "s/.*/${DIM}&${RESET}/" || return 1
}


function dna::portable_copy() {
    # Copy function using rsync with backup functionality
    # Arguments: source destination
    local source="$1"
    local destination="$2"
    local super_project_root="${3:-$(pwd)}"

    # Use rsync with backup functionality (no --update flag to preserve existing files)
    local rsync_flags=()
#    rsync_flags+=(--progress)
    rsync_flags+=(--verbose)
    rsync_flags+=(--backup --suffix='.old')

    if [[ -d "${source}" ]]; then
        # For directories, ensure trailing slash for proper rsync behavior
        dna::dimmed_rsync "${rsync_flags[@]}" --recursive "${source%/}/" "${destination}"
    else
        # For files
        dna::dimmed_rsync "${rsync_flags[@]}" "${source}" "${destination}"
    fi
    echo

    # Validate file ownership and permissions match the super project
    dna::validate_file_ownership_and_permissions "${destination}" "${super_project_root}" || return 1

    return 0
}

function dna::get_owner() {
    # Cross platform implementation
    if [[ "$(uname)" == "Darwin" ]]; then
        stat -f '%Su' "$1"
    else
        stat -c '%U' "$1"
    fi
}

function dna::get_group() {
    # Cross platform implementation
    if [[ "$(uname)" == "Darwin" ]]; then
        stat -f '%Su' "$1"
    else
        stat -c '%U' "$1"
    fi
}

function dna::get_permission() {
    # Cross platform implementation
    if [[ "$(uname)" == "Darwin" ]]; then
        stat -f '%A' "$1"
    else
        stat -c '%a' "$1"
    fi
}


function dna::validate_file_ownership_and_permissions() {
    # Validate that copied files/directories have ownership and permissions matching the super project
    # Arguments: target_path super_project_root
    local target_path="$1"
    local super_project_root="$2"

    # Get super project ownership and permissions
    local super_project_owner
    local super_project_group

    super_project_owner=$(dna::get_owner "${super_project_root}" 2>/dev/null)
    super_project_group=$(dna::get_group "${super_project_root}" 2>/dev/null)

    # Recursively fix ownership and permissions for the target path
    if [[ -d "$target_path" ]]; then
        # For directories, apply to all contents
        find "$target_path" -type f -exec chown "${super_project_owner}:${super_project_group}" {} \; 2>/dev/null || true
        find "$target_path" -type d -exec chown "${super_project_owner}:${super_project_group}" {} \; 2>/dev/null || true
        find "$target_path" -type f -exec chmod 644 {} \; 2>/dev/null || true
        find "$target_path" -type d -exec chmod 755 {} \; 2>/dev/null || true
    else
        # For files
        chown "${super_project_owner}:${super_project_group}" "$target_path" 2>/dev/null || true
        chmod 644 "$target_path" 2>/dev/null || true
    fi

    return 0
}


function dna::init_command() {
    local super_project_root
    super_project_root=$(pwd)

    # rsync is now a required dependency and should be available

    # ....cli......................................................................................
    while [[ $# -gt 0 ]]; do
        case "$1" in
            --help|-h)
                dna::command_help_menu "${DOCUMENTATION_BUFFER_INIT:?err}"
                exit 0
                ;;
            *)
                dna::unknown_subcommand_msg "init" "$*"
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

    if [[ "${super_project_root}" == "${DNA_ROOT:?err}" ]]; then
      n2st::print_msg_warning "Can't initialize the DNA repository (PWD=$(pwd)).\nAborting now!"
      return 1
    fi

    # ====Begin====================================================================================
    # Splash type: small, negative or big
    n2st::norlab_splash "${DNA_SPLASH_NAME_SMALL:?err}" "${DNA_GIT_REMOTE_URL}" "small"
    n2st::print_formated_script_header "init procedure" "${MSG_LINE_CHAR_BUILDER_LVL2}" "${MSG_LINE_STYLE_LVL2}"


    local project_git_remote_url
    local super_project_name
    local super_project_user
    local repo_top_dir_name
    repo_top_dir_name="$( basename "${super_project_root}" )"
    project_git_remote_url="$( git remote get-url origin )" || return 1
    super_project_name="$( basename "${project_git_remote_url}" .git )"
    super_project_user="$(id -un)"

    # ★ Note: Keep 'sudo', its required for preserving user interaction flow
    n2st::print_msg "Preparing ${super_project_name} DNA-initialization

Target path:
${MSG_DIMMED_FORMAT}
     ${super_project_root}
${MSG_END_FORMAT}
Current repository structure:
${MSG_DIMMED_FORMAT}
$(sudo tree -L 1 -a --noreport --dirsfirst -F -I .git -I .idea -I .cadence "${super_project_root}" | sed "s;^${super_project_root%/};${repo_top_dir_name};" | sed 's/^/     /')
${MSG_END_FORMAT}
DNA-initialisation will add the following:
${MSG_DIMMED_FORMAT}
     ${repo_top_dir_name}/
     ├── .dockerized_norlab/                 ← DNA configuration directory
     │   ├── configuration/                  ← Main configuration files
     │   │   ├── project_entrypoints/        ← Container startup scripts
     │   │   ├── project_requirements/       ← Dependency specifications
     │   │   ├── Dockerfile                  ← Container build instructions
     │   │   ├── .env                        ← Project environment variables
     │   │   ├── .env.dna                    ← DNA-specific variables
     │   │   ├── .env.local                  ← Local development overrides
     │   │   └── README.md                   ← Configuration documentation
     │   ├── dn_container_env_variable/      ← Container environment exports
     │   ├── .env.${super_project_name}
     │   └── README.md                       ← DNA configuration quick documentation
     ├── artifact/                           ← Runtime produced data (mounted)
     ├── external_data/                      ← Pre-existing data (mounted)
     ├── src/                                ← Your source code (mounted/copied)
     ├── tests/                              ← Your test code (mounted/copied)
     ├── .dockerignore                       ← Docker build exclusions
     ├── .gitignore                          ← Git exclusions
     └── README.md                           ← Project documentation
${MSG_END_FORMAT}"

    # Check if .dockerized_norlab already exists
    if [[ -d ".dockerized_norlab" ]]; then
        unset user_input
        n2st::print_msg_warning "This project is already DNA initialized since ${MSG_DIMMED_FORMAT}.dockerized_norlab${MSG_END_FORMAT} directory already exists.\nIf you continue, existing file and directories with the same name will be safeguarded with the suffix '.old', not overriden."
        read -r -n 1 -p "Do you want to continue [y/N]" user_input
        if [[ "${user_input}" == "y" || "${user_input}" == "Y" ]]; then
          echo
        else
          n2st::print_msg "No problem, see you later"
          return 0
        fi
    else
        n2st::print_msg "Ready to proceed with DNA-initialization"
        unset user_input
        read -r -n 1 -p "Execute? [y/N]" user_input
        if [[ "${user_input}" == "y" || "${user_input}" == "Y" ]]; then
          echo
        else
          n2st::print_msg "No problem, see you later"
          return 0
        fi
    fi

    echo
    n2st::print_msg "Initializing ${super_project_name}..."
    echo

    mkdir -p "${super_project_root}/.dockerized_norlab" || return 1

    # Copy template files
    dna::portable_copy "${DNA_LIB_PATH}/template/.dockerized_norlab/" .dockerized_norlab "${super_project_root}" || return 1

    #tree -L 2 -a "$PWD"  >&3 # (CRITICAL) ToDo: on task end >> delete this line ←

    cd "${super_project_root}/.dockerized_norlab" || return 1

    # Rename the super project DNA meta .env file
    mv -f "${super_project_root}/.dockerized_norlab/.env.PLACEHOLDER_SUPER_PROJECT_NAME" ".env.${super_project_name}" || return 1

    # Cleanup 'dn_container_env_variable/' content
    rm -f "dn_container_env_variable/.env.dn_expose_PLACEHOLDER_DN_CONTAINER_NAME"

    # Replace placeholders in the .env.dna file
    cd "${super_project_root}/.dockerized_norlab/configuration/" || return 1
    local super_project_acronym
    super_project_acronym="$(dna::get_super_project_acronym "${super_project_name}")"

    {
      n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_PROJECT_GIT_REMOTE_URL" "${project_git_remote_url}" ".env.dna" &&
      n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_CONTAINER_NAME" "IamDNA_${super_project_acronym}" ".env.dna" &&
      n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_PROJECT_ALIAS_PREFIX" "${super_project_acronym}" ".env.dna"
    } || return 1

    ## Replace placeholders in the DNA readme file
    #cd "${super_project_root}/.dockerized_norlab/" || return 1
    #{
    #  n2st::seek_and_modify_string_in_file "PLACEHOLDER_DN_CONTAINER_NAME" "IamDNA_${super_project_acronym}" "README.md" &&
    #  n2st::seek_and_modify_string_in_file "PLACEHOLDER_SUPER_PROJECT_USER" "${super_project_user}" "README.md"
    #}  || return 1
    #
    ## Note: There is three ocurence
    #{
    #  n2st::seek_and_modify_string_in_file "PLACEHOLDER_SUPER_PROJECT_NAME" "${super_project_name}" "README.md" &&
    #  n2st::seek_and_modify_string_in_file "PLACEHOLDER_SUPER_PROJECT_NAME" "${super_project_name}" "README.md" &&
    #  n2st::seek_and_modify_string_in_file "PLACEHOLDER_SUPER_PROJECT_NAME" "${super_project_name}" "README.md"
    #} || return 1

    # ....Create root repository required directories..............................................
    cd "${super_project_root}" || return 1

    {
      mkdir -p artifact/optuna_storage &&
      mkdir -p external_data &&
      mkdir -p src/launcher/configs &&
      mkdir -p src/dna_example &&
      mkdir -p tests/test_dna_example
    } || return 1

    dna::portable_copy "${DNA_LIB_PATH}/template/artifact/README.md" artifact/ "${super_project_root}" || return 1
    dna::portable_copy "${DNA_LIB_PATH}/template/artifact/optuna_storage/README.md" artifact/optuna_storage/ "${super_project_root}" || return 1
    dna::portable_copy "${DNA_LIB_PATH}/template/external_data/README.md" external_data/ "${super_project_root}" || return 1
    dna::portable_copy "${DNA_LIB_PATH}/template/src/launcher/" src/launcher/ "${super_project_root}" || return 1
    dna::portable_copy "${DNA_LIB_PATH}/template/src/dna_example/" src/dna_example/ "${super_project_root}" || return 1
    if [[ ! -f "src/README.md" ]]; then
      dna::portable_copy "${DNA_LIB_PATH}/template/src/README.md" src/ "${super_project_root}" || return 1
    fi
    dna::portable_copy "${DNA_LIB_PATH}/template/tests/" tests/ "${super_project_root}" || return 1

    # ....Create root README.md files if it does't exist...........................................
    cd "${super_project_root}" || return 1

    if [[ ! -f "README.md" ]]; then
        touch "README.md"  || return 1
        cat > "README.md" << EOF
# ${super_project_name}

This project is initialized with [${DNA_HUMAN_NAME:?err} (DNA)](${DNA_GIT_REMOTE_URL:?err}).
See '.dockerized_norlab/README.md' for usage details.

EOF
    fi

    # ....Setup ignore files.......................................................................
    cd "${super_project_root}" || return 1

    if [[ ! -f ".gitignore" ]]; then
        # Case: file does not exist => copy template
        dna::portable_copy "${DNA_LIB_PATH}/template/.gitignore" .gitignore "${super_project_root}" || return 1
    else
        # Case: file exist => append required .gitignore entries
        cat >> ".gitignore" << EOF

# ====Dockerized-NorLab(required)==================================================================
**/.dockerized_norlab/dn_container_env_variable/
**/.dockerized_norlab/configuration/.env.local

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
        dna::portable_copy "${DNA_LIB_PATH}/template/.dockerignore" .dockerignore "${super_project_root}" || return 1
    else
        # Case: file exist => append required .gitignore entries
      cat >> ".dockerignore" << EOF

# ====Dockerized-NorLab(required)==================================================================
!**/.dockerized_norlab/
!**/version.txt
!**/.git

# ====Dockerized-NorLab(recommended)===============================================================
**/external_data/
**/artifact/
**/slurm_jobs_logs/*.log

EOF
    fi

    n2st::draw_horizontal_line_across_the_terminal_window "─" "$(tput dim)"


    # ....Validate init procedure..................................................................
    cd "${super_project_root}" || return 1
    source "${DNA_LIB_PATH}/core/utils/super_project_dna_sanity_check.bash" || return 1

    # ....Setup host procedure.....................................................................
    source "${DNA_LIB_PATH}/core/utils/setup_host_for_running_this_super_project.bash" || return 1

    # ====Teardown=================================================================================
    cd "${super_project_root}" || return 1
    echo
    n2st::print_msg_done "DNA project initialized successfully.

New repository structure
${MSG_DIMMED_FORMAT}
$(tree -L 1 -a --noreport --dirsfirst -F -I .git -I .idea -I .cadence "${super_project_root}" | sed "s;^${super_project_root%/};${repo_top_dir_name};" | sed 's/^/     /')
${MSG_END_FORMAT}

Content of new ${MSG_EMPH_FORMAT}.dockerized_norlab${MSG_END_FORMAT} directory
${MSG_DIMMED_FORMAT}
$(tree -L 2 -a --noreport --dirsfirst -F -I .git -I visual ".dockerized_norlab" | sed 's/^/     /')
${MSG_END_FORMAT}

You can now use ${MSG_EMPH_FORMAT}dna${MSG_END_FORMAT} to manage your project.
To get started:
  1. Execute ${MSG_EMPH_FORMAT}dna help${MSG_END_FORMAT} to see available command
  2. Check 'Project Initialization & Configuration' section in 'documentation'
     at https://github.com/norlab-ulaval/dockerized-norlab-project
  3. If you are in a hurry, read section 'Getting started ... fast'
     in ${MSG_DIMMED_FORMAT}${super_project_name}/.dockerized_norlab/README.md${MSG_END_FORMAT}


$(n2st::echo_centering_str "Stay awesome 🦾" ' ' ' ')"
    n2st::print_formated_script_footer "init procedure" "${MSG_LINE_CHAR_BUILDER_LVL2}" "${MSG_LINE_STYLE_LVL2}"
    cd "${super_project_root}" || return 1
    return 0
}
