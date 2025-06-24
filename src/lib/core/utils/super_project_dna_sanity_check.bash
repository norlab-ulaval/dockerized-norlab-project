#!/bin/bash
# =================================================================================================
# Check that Dockerized-Norlab-Project components and dependencies are installed at the required
# location in the super-project
#
# Usage:
#   $ bash super_project_dna_sanity_check.bash
#   or
#   $ source super_project_dna_sanity_check.bash && dna::super_project_dna_sanity_check
#
# Global:
#  read DNA_ROOT
#  read SUPER_PROJECT_ROOT
#  read SUPER_PROJECT_REPO_NAME
#
# =================================================================================================

function dna::super_project_dna_sanity_check() {
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
  dna::check_super_project_dir_structure

  # ....check .dockerized_norlab directory structure.......................................
  dna::check_dockerized_project_configuration_dir_structure
  dna::check_project_configuration
  dna::check_project_entrypoints

  # ....check .gitignore files entries.............................................................
  dna::check_gitignore

  # ....check .dockerignore files entries..........................................................
  dna::check_dockerignore

  #  ....Teardown...................................................................................
  echo -e "${MSG_DONE_FORMAT}[DNA done]${MSG_END_FORMAT} Super project ${SUPER_PROJECT_REPO_NAME:?err} setup is OK"
  cd "${tmp_cwd}" || { echo "Return to original dir error" 1>&2 && return 1; }
  return 0
}

function dna::check_super_project_dir_structure() {
  test -d ".dockerized_norlab" || n2st::print_msg_error_and_exit "'.dockerized_norlab' is not installed at super-project repository root as required!"
  test -d "src/" || n2st::print_msg_error_and_exit "The 'src' directory is not installed at super-project repository root as required!"
  test -d "tests/" || n2st::print_msg_error_and_exit "The 'tests' directory is not installed at super-project repository root as required!"
  test -d "slurm_jobs/" || n2st::print_msg_error_and_exit "The 'slurm_jobs/' directory is not installed as required!"
  test -d "external_data/" || n2st::print_msg_error_and_exit "The 'external_data' directory is not installed at super-project repository root as required!"
  test -d "artifact/" || n2st::print_msg_error_and_exit "The 'artifact' directory is not installed at super-project repository root as required!"
  test -d "artifact/optuna_storage/" || n2st::print_msg_error_and_exit "The 'optuna_storage' directory is not installed in the super-project 'artifact/' directory as required!"
  test -f ".gitignore" || n2st::print_msg_error_and_exit "'.gitignore' is not installed at super-project repository root as required!"
  test -f ".dockerignore" || n2st::print_msg_error_and_exit "'.dockerignore' is not installed at super-project repository root as it should!"
}

function dna::check_dockerized_project_configuration_dir_structure() {
  test_dir_path=".dockerized_norlab"
  cd "${SUPER_PROJECT_ROOT}/${test_dir_path}" || exit 1
  test -d "configuration/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/configuration/' directory is not installed as required!"
  test -d "dn_container_env_variable/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/dn_container_env_variable/' directory is not installed as required!"
}

function dna::check_project_configuration() {
  test_dir_path=".dockerized_norlab/configuration"
  cd "${SUPER_PROJECT_ROOT}/${test_dir_path}" || exit 1
  test -d "project_requirements/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project_requirements/' directory is not installed as required!"
  test -d "project_entrypoints/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project_entrypoints/' directory is not installed as required!"
  test -f ".env.dna" || n2st::print_msg_error_and_exit "The '${test_dir_path}/.env.dna' file is not installed as required!"
  test -f ".env" || n2st::print_msg_error_and_exit "The '${test_dir_path}/.env' file is not installed as required!"
  test -f ".env.local" || n2st::print_msg_error_and_exit "The '${test_dir_path}/.env.local' file is not installed as required!"
  test -f "Dockerfile" || n2st::print_msg_error_and_exit "The '${test_dir_path}/Dockerfile' file is not installed as required!"
}

function dna::check_project_entrypoints() {
  test_dir_path=".dockerized_norlab/configuration/project_entrypoints"
  cd "${SUPER_PROJECT_ROOT}/${test_dir_path}" || exit 1
  test -d "project-ci-tests/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project-ci-tests/' directory is not installed as required!"
  test -d "project-ci-tests/test_jobs/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project-ci-tests/test_jobs/' directory is not installed as required!"
  test -d "project-deploy/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project-deploy/' directory is not installed as required!"
  test -d "project-develop/" || n2st::print_msg_error_and_exit "The '${test_dir_path}/project-develop/' directory is not installed as required!"
}

function dna::check_gitignore() {
  cd "${SUPER_PROJECT_ROOT}" || exit 1

  # Check required entry: **/.dockerized_norlab/dn_container_env_variable/
  if ! grep --silent -E "^\*\*\/\.dockerized_norlab\/dn_container_env_variable\/$" ".gitignore"; then
    n2st::print_msg_error_and_exit "The line '**/.dockerized_norlab/dn_container_env_variable/' is not present in .gitignore as required!"
  fi
  # Check required entry: **/.dockerized_norlab/configuration/.env.local
  if ! grep --silent -E "^\*\*\/\.dockerized_norlab\/configuration\/\.env.local$" ".gitignore"; then
    n2st::print_msg_error_and_exit "The line '**/.dockerized_norlab/configuration/.env.local' is not present in .gitignore as required!"
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

function dna::check_dockerignore() {
  cd "${SUPER_PROJECT_ROOT}" || exit 1

  # Check required entry: !**/.dockerized_norlab/
  if ! grep --silent -E "^\!\*\*\/\.dockerized_norlab\/$" ".dockerignore"; then
    n2st::print_msg_error_and_exit "The line '!**/.dockerized_norlab/' is not present in .dockerignore as it should be!"
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
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  # This script is being run, ie: __name__="__main__"

  # ....Source project shell-scripts dependencies..................................................
  script_path="$(realpath -q "${BASH_SOURCE[0]:-.}")"
  script_path_parent="$(dirname "${script_path}")"
  source "${script_path_parent}/import_dna_lib.bash" || exit 1
  source "${script_path_parent}/load_super_project_config.bash" || exit 1

  # ....Execute....................................................................................
  n2st::norlab_splash "${DNA_SPLASH_NAME_FULL:?err}" "${DNA_GIT_REMOTE_URL:?err}" "negative"
  n2st::print_formated_script_header "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  dna::super_project_dna_sanity_check || exit 1
  fct_exit_code=$?
  n2st::print_formated_script_footer "$(basename $0)" "${MSG_LINE_CHAR_BUILDER_LVL1}"
  exit "${fct_exit_code}"
else
  # This script is being sourced, ie: __name__="__source__"

  # ....Pre-condition..............................................................................
  dna_error_prefix="\033[1;31m[DNA error]\033[0m"
  test -n "$( declare -f dna::import_lib_and_dependencies )" || { echo -e "${dna_error_prefix} The DNA lib is not loaded!" 1>&2 && exit 1; }
  test -n "$( declare -f n2st::print_msg )" || { echo -e "${dna_error_prefix} The N2ST lib is not loaded!" 1>&2 && exit 1; }
  test -d "${DNA_ROOT:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }
  test -d "${DNA_LIB_PATH:?err}" || { echo -e "${dna_error_prefix} library load error!" 1>&2 && exit 1; }

fi
