#!/bin/bash
# =================================================================================================
# Check that Dockerized-Norlab-Project components and dependencies are installed at the required
# location in the super-project
#
# Usage:
#   $ bash super_project_dnp_sanity_check.bash
#   or
#   $ source super_project_dnp_sanity_check.bash && dnp::super_project_dnp_sanity_check
#
# Global:
#  read DNP_ROOT
#  read SUPER_PROJECT_ROOT
#  read SUPER_PROJECT_REPO_NAME
#
# =================================================================================================

function dnp::super_project_dnp_sanity_check() {
  # ....Setup......................................................................................
  local tmp_cwd
  tmp_cwd=$(pwd)

  if [[ -z "${SUPER_PROJECT_ROOT}" ]] || [[ -z "${SUPER_PROJECT_REPO_NAME}" ]]; then
    n2st::print_msg_error_and_exit "Super project configs are not loaded, run load_super_project_config.bash first!"
  fi

  if [[ ! -d "${SUPER_PROJECT_ROOT}" ]]; then
    n2st::print_msg_error_and_exit "Super project root is unreachable!"
  fi

  # ====Begin======================================================================================
  cd "${SUPER_PROJECT_ROOT:?err}" || exit 1

  if [[ $( git -C "${SUPER_PROJECT_ROOT}" rev-parse --is-inside-work-tree 2>/dev/null ) != true ]]; then
    n2st::print_msg_error_and_exit "The '.git' directory does not exist at super-project repository root as required!
Dockerized-NorLab-Porject require that the super project be under version control using git."
  else
    git status >/dev/null || exit 1
  fi

  # ....check super project directory structure....................................................
  dnp::check_super_project_dir_structure

  # ....check .dockerized_norlab_project directory structure.......................................
  dnp::check_dockerized_project_configuration_dir_structure
  dnp::check_project_configuration
  dnp::check_project_entrypoints

  # ....check .gitignore files entries.............................................................
  dnp::check_gitignore

  # ....check .dockerignore files entries..........................................................
  dnp::check_dockerignore

  #  ....Teardown...................................................................................
  echo -e "${MSG_DONE_FORMAT}[DNP done]${MSG_END_FORMAT} Super project ${SUPER_PROJECT_REPO_NAME:?err} setup is OK"
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

function dnp::check_super_project_dir_structure() {
  test -d ".dockerized_norlab_project" || n2st::print_msg_error_and_exit "'.dockerized_norlab_project' is not installed at super-project repository root as required!"
  test -d "src/" || n2st::print_msg_error_and_exit "The 'src' directory is not installed at super-project repository root as required!"
  test -d "tests/" || n2st::print_msg_error_and_exit "The 'tests' directory is not installed at super-project repository root as required!"
  test -d "external_data/" || n2st::print_msg_error_and_exit "The 'external_data' directory is not installed at super-project repository root as required!"
  test -d "artifact/" || n2st::print_msg_error_and_exit "The 'artifact' directory is not installed at super-project repository root as required!"
  test -d "artifact/optuna_storage/" || n2st::print_msg_error_and_exit "The 'optuna_storage' directory is not installed in the super-project 'artifact/' directory as required!"
  test -f ".gitignore" || n2st::print_msg_error_and_exit "'.gitignore' is not installed at super-project repository root as required!"
  test -f ".dockerignore" || n2st::print_msg_error_and_exit "'.dockerignore' is not installed at super-project repository root as it should!"
}

function dnp::check_dockerized_project_configuration_dir_structure() {
  test_dir_path=".dockerized_norlab_project"
  cd "${SUPER_PROJECT_ROOT}/${test_dir_path}" || exit 1
  test -d "configuration/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/configuration/' directory is not installed as required!"
  test -d "dn_container_env_variable/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/dn_container_env_variable/' directory is not installed as required!"
  test -d "slurm_jobs/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/slurm_jobs/' directory is not installed as required!"
}

function dnp::check_project_configuration() {
  test_dir_path=".dockerized_norlab_project/configuration"
  cd "${SUPER_PROJECT_ROOT}/${test_dir_path}" || exit 1
  test -d "project_requirements/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project_requirements/' directory is not installed as required!"
  test -d "project_entrypoints/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project_entrypoints/' directory is not installed as required!"
  test -f ".env.dnp" || n2st::print_msg_error_and_exit "The '${test_dir_path}/.env.dnp' file is not installed as required!"
  test -f ".env" || n2st::print_msg_error_and_exit "The '${test_dir_path}/.env' file is not installed as required!"
  test -f ".env.local" || n2st::print_msg_error_and_exit "The '${test_dir_path}/.env.local' file is not installed as required!"
  test -f "Dockerfile" || n2st::print_msg_error_and_exit "The '${test_dir_path}/Dockerfile' file is not installed as required!"
}

function dnp::check_project_entrypoints() {
  test_dir_path=".dockerized_norlab_project/configuration/project_entrypoints"
  cd "${SUPER_PROJECT_ROOT}/${test_dir_path}" || exit 1
  test -d "project-ci-tests/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project-ci-tests/' directory is not installed as required!"
  test -d "project-ci-tests/test_jobs/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project-ci-tests/test_jobs/' directory is not installed as required!"
  test -d "project-deploy/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project-deploy/' directory is not installed as required!"
  test -d "project-develop/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project-develop/' directory is not installed as required!"
}

function dnp::check_gitignore() {
  cd "${SUPER_PROJECT_ROOT}" || exit 1

  # Check required entry: **/.dockerized_norlab_project/dn_container_env_variable/
  if ! grep --silent -E "^\*\*\/\.dockerized_norlab_project\/dn_container_env_variable\/$" ".gitignore"; then
    n2st::print_msg_error_and_exit "The line '**/.dockerized_norlab_project/dn_container_env_variable/' is not present in .gitignore as required!"
  fi
  # Check required entry: **/.dockerized_norlab_project/configuration/.env.local
  if ! grep --silent -E "^\*\*\/\.dockerized_norlab_project\/configuration\/\.env.local$" ".gitignore"; then
    n2st::print_msg_error_and_exit "The line '**/.dockerized_norlab_project/configuration/.env.local' is not present in .gitignore as required!"
  fi
  # Check recommended entry: **/external_data/
  if ! grep --silent -E "^\*\*\/external_data\/$" ".gitignore"; then
    n2st::print_msg_warning "The line '**/external_data/' is not present in .gitignore as recommended!"
  fi
  # Check recommended entry: **/artifact/
  if ! grep --silent -E "^\*\*\/artifact\/$" ".gitignore"; then
    n2st::print_msg_warning "The line '**/artifact/' is not present in .gitignore as recommended!"
  fi
}

function dnp::check_dockerignore() {
  cd "${SUPER_PROJECT_ROOT}" || exit 1

  # Check required entry: !**/.dockerized_norlab_project/
  if ! grep --silent -E "^\!\*\*\/\.dockerized_norlab_project\/$" ".dockerignore"; then
    n2st::print_msg_error_and_exit "The line '!**/.dockerized_norlab_project/' is not present in .dockerignore as it should be!"
  fi
  # Check required entry: !**/version.txt
  if ! grep --silent -E "^\!\*\*\/version.txt$" ".dockerignore"; then
    n2st::print_msg_error_and_exit "The line '!**/version.txt' is not present in .dockerignore as it should be!"
  fi
  # Check required entry: !**/.git
  if ! grep --silent -E "^\!\*\*\/\.git$" ".dockerignore"; then
    n2st::print_msg_error_and_exit "The line '!**/.git' is not present in .dockerignore as it should be!"
  fi
    # Check recommended entry: **/external_data/
  if ! grep --silent -E "^\*\*\/external_data\/$" ".dockerignore"; then
    n2st::print_msg_warning "The line '**/external_data/' is not present in .dockerignore as recommended!"
  fi
  # Check recommended entry: **/artifact/
  if ! grep --silent -E "^\*\*\/artifact\/$" ".dockerignore"; then
    n2st::print_msg_warning "The line '**/artifact/' is not present in .dockerignore as recommended!"
  fi
}

# ::::Main:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
if [[ "${BASH_SOURCE[0]}" = "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath "${BASH_SOURCE[0]:-'.'}")"
  script_path_parent="$(dirname "${script_path}")"
  source "${script_path_parent}/import_dnp_lib.bash" || exit 1
  source "${script_path_parent}/load_super_project_config.bash" || exit 1

  # ....Execute....................................................................................
  n2st::norlab_splash "${DNP_GIT_NAME:?err} (${DNP_PROMPT_NAME:?err})" "${DNP_GIT_REMOTE_URL:?err}"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dnp::super_project_dnp_sanity_check || exit 1
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dnp_error_prefix="\033[1;31m[DNP error]\033[0m"
  test -n "$( declare -F dnp::import_lib_and_dependencies )" || { echo -e "${dnp_error_prefix} The DNP lib is not loaded!" ; exit 1 ; }
  test -n "$( declare -F n2st::print_msg )" || { echo -e "${dnp_error_prefix} The N2ST lib is not loaded!" ; exit 1 ; }
  test -d "${DNP_ROOT:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }
  test -d "${DNP_LIB_PATH:?err}" || { echo -e "${dnp_error_prefix} librairy load error!" ; exit 1 ; }

fi
