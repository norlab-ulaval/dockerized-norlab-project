#!/usr/bin/env bats
# =================================================================================================
# Usage in docker container
#   $ REPO_ROOT=$(pwd) && RUN_TESTS_IN_DIR='tests'
#   $ docker run -it --rm -v "$REPO_ROOT:/code" bats/bats:latest "$RUN_TESTS_IN_DIR"
#
#   Note: "/code" is the working directory in the bats official image
#
# bats-core ref:
#   - https://bats-core.readthedocs.io/en/stable/tutorial.html
#   - https://bats-core.readthedocs.io/en/stable/writing-tests.html
#   - https://opensource.com/article/19/2/testing-bash-bats
#       ↳ https://github.com/dmlond/how_to_bats/blob/master/test/build.bats
#
# Helper library:
#   - https://github.com/bats-core/bats-assert
#   - https://github.com/bats-core/bats-support
#   - https://github.com/bats-core/bats-file
#
# =================================================================================================

bats_path=/usr/lib/bats
error_prefix="[\033[1;31mN2ST ERROR\033[0m]"
if [[ -d ${bats_path} ]]; then
  # ....Bats-core recommended helper functions.....................................................
  load "${bats_path}/bats-support/load"
  load "${bats_path}/bats-assert/load"
  load "${bats_path}/bats-file/load"
  # ....Optional...................................................................................
  #load "${bats_path}/bats-detik/load" # <- Kubernetes support
  # ....N2ST library helper function...............................................................
  load "${SRC_CODE_PATH:?err}/${N2ST_BATS_TESTING_TOOLS_RELATIVE_PATH:?err}/bats_helper_functions"
  load "${SRC_CODE_PATH}/tests/tests_bats/bats_testing_tools/bats_helper_functions_local"
else
  echo -e "\n${error_prefix} $0 path to bats-core helper library unreachable at \"${bats_path}\"!"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi

# ====Tests file configuration=====================================================================

TESTED_FILE="install.bash"
TESTED_FILE_PATH=""  # Root of the repository

# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR

  # Create temporary directory for tests
  # Note: Copy "dockerized-norlab-project/*" including the root dir original name to prevent
  #       container src code override by mocking function, which would brake following .bats tests.
  export TEMP_DIR=$(temp_make)
  export TEMP_DNP_DIR="${TEMP_DIR}/$(basename "${BATS_DOCKER_WORKDIR}")"
  cp -R "${BATS_DOCKER_WORKDIR}" "${TEMP_DNP_DIR}/"

#  tree -L 4 -a "$BATS_DOCKER_WORKDIR/src"  >&3

  # Create a mock import_norlab_shell_script_tools_lib.bash
  cat > "${TEMP_DNP_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash" << 'EOF'
#!/bin/bash
# Mock import_norlab_shell_script_tools_lib.bash

tmp_pwd=$(pwd)
cd ${TEMP_DNP_DIR}/utilities/norlab-shell-script-tools/src/function_library/
source general_utilities.bash
cd $tmp_pwd


# Set message formatting variables
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""
export MSG_LINE_CHAR_BUILDER_LVL1="-"

# Mock N2ST functions
function n2st::print_msg() {
  echo "Mock n2st::print_msg called with args: $*"
  return 0
}

function n2st::print_msg_warning() {
  echo "Mock n2st::print_msg_warning called with args: $*"
  return 0
}

function n2st::print_msg_error() {
  echo "Mock n2st::print_msg_error called with args: $*"
  return 0
}

function n2st::print_msg_error_and_exit() {
  echo "Mock n2st::print_msg_error_and_exit called with args: $*"
  exit 1
}

function n2st::print_msg_done() {
  echo "Mock n2st::print_msg_done called with args: $*"
  return 0
}

function n2st::print_formated_script_header() {
  echo "Mock n2st::print_formated_script_header called with args: $*"
  return 0
}

function n2st::print_formated_script_footer() {
  echo "Mock n2st::print_formated_script_footer called with args: $*"
  return 0
}

function n2st::norlab_splash() {
  echo "Mock n2st::norlab_splash called with args: $*"
  return 0
}

# Export mock functions
for func in $(compgen -A function | grep -e n2st::); do
  # shellcheck disable=SC2163
  export -f "${func}"
done

# Print a message to indicate that the mock import_norlab_shell_script_tools_lib.bash has been loaded
echo "[N2ST done] Mock import_norlab_shell_script_tools_lib.bash loaded"
EOF

  # Create a mock ui.bash
  cat > "${TEMP_DNP_DIR}/src/lib/core/utils/ui.bash" << 'EOF'
#!/bin/bash
# Mock ui.bash

function dnp::command_help_menu() {
  echo "Mock dnp::command_help_menu called with args: $*"
  return 0
}

function dnp::unknown_option_msg() {
  echo "Mock dnp::unknown_option_msg called with args: $*"
  return 1
}

# Export mock functions
for func in $(compgen -A function | grep -e dnp::); do
  # shellcheck disable=SC2163
  export -f "${func}"
done

# Print a message to indicate that the mock ui.bash has been loaded
echo "[DNP done] Mock ui.bash loaded"
EOF

  # Create a mock setup_host_dnp_requirements.bash
  cat > "${TEMP_DNP_DIR}/src/lib/core/utils/setup_host_dnp_requirements.bash" << 'EOF'
#!/bin/bash
# Mock setup_host_dnp_requirements.bash

echo "Mock setup_host_dnp_requirements.bash executed"
exit 0
EOF
}

setup() {
  cd "${TEMP_DNP_DIR}" || exit 1
}

# ....Teardown.....................................................................................
teardown() {
  bats_print_run_env_variable_on_error
  rm -f /usr/local/bin/dnp
  sed -i '/# >>>> Dockerized-NorLab-Project (start)/,/# <<<< Dockerized-NorLab-Project (end)/d' "${HOME}/.bashrc"
  cd "${TEMP_DNP_DIR}" || exit 1
}

teardown_file() {
  # Clean up temporary directories
  temp_del "${TEMP_DIR}"
  #tree -L 2 -a $PWD >&3
}

# ====Test cases==================================================================================

# Test case for help option
@test "dnp::install_dockerized_norlab_project_on_host with --help › expect help menu" {
  # Test case: When install.bash is called with --help, it should show the help menu
  run bash -c "source ${TEMP_DNP_DIR}/install.bash && dnp::install_dockerized_norlab_project_on_host --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::install_dockerized_norlab_project_on_host with -h › expect help menu" {
  # Test case: When install.bash is called with -h, it should show the help menu
  run bash -c "source ${TEMP_DNP_DIR}/install.bash && dnp::install_dockerized_norlab_project_on_host -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dnp::command_help_menu called with args:"
}

@test "dnp::install_dockerized_norlab_project_on_host with unknown option › expect error" {
  # Test case: When install.bash is called with an unknown option, it should show an error
  run bash -c "source ${TEMP_DNP_DIR}/install.bash && dnp::install_dockerized_norlab_project_on_host --unknown-option"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dnp::unknown_option_msg called with args: bash ./install.bash"
}

@test "dnp::install_dockerized_norlab_project_on_host with default options › expect system-wide symlink" {
  # Test case: When install.bash is called with default options, it should create a system-wide symlink
  run bash -c "source ${TEMP_DNP_DIR}/install.bash && dnp::install_dockerized_norlab_project_on_host"

  # Should succeed
  assert_success

  assert_file_executable "${TEMP_DNP_DIR}/src/bin/dnp"
  assert_symlink_to "${TEMP_DNP_DIR}/src/bin/dnp" "/usr/local/bin/dnp"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNP_PATH=.*$"

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Setting up host for Dockerized-NorLab-Project"
  assert_output --partial "Mock n2st::print_msg called with args: Creating symlink:"
  assert_output --partial "Mock n2st::print_msg_done called with args: Dockerized-NorLab-Project has been installed successfully"
  assert_output --partial "Mock n2st::print_msg called with args: You can now use 'dnp' command from anywhere"

}


@test "dnp::install_dockerized_norlab_project_on_host with --skip-system-wide-symlink-install › expect no symlink" {
  # Test case: When install.bash is called with --skip-system-wide-symlink-install, it should not create a symlink
  run bash -c "source ${TEMP_DNP_DIR}/install.bash && dnp::install_dockerized_norlab_project_on_host --skip-system-wide-symlink-install"

  # Should succeed
  assert_success

  # Should not create a symlink
  assert_file_executable "${TEMP_DNP_DIR}/src/bin/dnp"
  assert_not_symlink_to "${TEMP_DNP_DIR}/src/bin/dnp" "/usr/local/bin/dnp"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNP_PATH=.*$"
  refute_output --partial "Mock n2st::print_msg called with args: Creating symlink:"

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Setting up host for Dockerized-NorLab-Project"
  assert_output --partial "Mock n2st::print_msg_done called with args: Dockerized-NorLab-Project has been installed successfully"
  assert_output --partial "Mock n2st::print_msg called with args: You can use"
}

@test "dnp::install_dockerized_norlab_project_on_host with --add-dnp-path-to-bashrc › expect bashrc update" {
  # Test case: When install.bash is called with --add-dnp-path-to-bashrc, it should update ~/.bashrc
  run bash -c "source ${TEMP_DNP_DIR}/install.bash && dnp::install_dockerized_norlab_project_on_host --add-dnp-path-to-bashrc"

  # Should succeed
  assert_success

  # Should not create a symlink
  assert_file_executable "${TEMP_DNP_DIR}/src/bin/dnp"
  assert_not_symlink_to "${TEMP_DNP_DIR}/src/bin/dnp" "/usr/local/bin/dnp"
  assert_file_contains "${HOME}/.bashrc" "^export _DNP_PATH=.*$"
  refute_output --partial "Mock n2st::print_msg called with args: Creating symlink:"

  # Should update ~/.bashrc
  assert_output --partial "Mock n2st::print_msg called with args: Adding dnp entrypoint path to ~/.bashrc"

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Setting up host for Dockerized-NorLab-Project"
  assert_output --partial "Mock n2st::print_msg_done called with args: Dockerized-NorLab-Project has been installed successfully"
  assert_output --partial "Mock n2st::print_msg called with args: After restarting your shell or sourcing ~/.bashrc"
}

@test "dnp::install_dockerized_norlab_project_on_host with --yes › expect non-interactive installation" {
  # Test case: When install.bash is called with --yes, it should perform a non-interactive installation
  run bash -c "source ${TEMP_DNP_DIR}/install.bash && dnp::install_dockerized_norlab_project_on_host --yes"

  # Should succeed
  assert_success

  assert_file_executable "${TEMP_DNP_DIR}/src/bin/dnp"
  assert_symlink_to "${TEMP_DNP_DIR}/src/bin/dnp" "/usr/local/bin/dnp"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNP_PATH=.*$"

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Setting up host for Dockerized-NorLab-Project"
  assert_output --partial "Mock n2st::print_msg called with args: Creating symlink:"
  assert_output --partial "Mock n2st::print_msg_done called with args: Dockerized-NorLab-Project has been installed successfully"
  assert_output --partial "Mock n2st::print_msg called with args: You can now use 'dnp' command from anywhere"
}

@test "dnp::create_bin_dnp_to_entrypoint_symlink › expect symlink created" {
  # Test case: When dnp::create_bin_dnp_to_entrypoint_symlink is called, it should create a symlink
  local dnp_entrypoint="${TEMP_DNP_DIR}/src/bin/dnp"
  local bin_dnp_path="${TEMP_DIR}/usr/local/bin/dnp"

  # Create the directory for the symlink
  mkdir -p "$(dirname "${bin_dnp_path}")"

  source "${TEMP_DNP_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNP_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNP_DIR}/install.bash"

  assert_not_symlink_to "${dnp_entrypoint}" "${bin_dnp_path}"

  run dnp::create_bin_dnp_to_entrypoint_symlink "${dnp_entrypoint}" "${bin_dnp_path}"

  # Should succeed
  assert_success
  assert_symlink_to "${dnp_entrypoint}" "${bin_dnp_path}"

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg called with args: Creating symlink:"

}

@test "dnp::update_bashrc_dnp_bin_path › expect bashrc updated" {
  # Test case: When dnp::update_bashrc_dnp_bin_path is called, it should update ~/.bashrc
  local dnp_bin_dir="${TEMP_DNP_DIR}/src/bin"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNP_PATH=.*$"

  # Create a ~/.bashrc file with existing DNP_PATH entries
  cat > "${HOME}/.bashrc" << EOF
# Existing .bashrc content
# >>>> Dockerized-NorLab-Project (start)
export _DNP_PATH="/old/path"
export PATH="\$PATH:\$_DNP_PATH"
# <<<< Dockerized-NorLab-Project (end)
EOF

  source "${TEMP_DNP_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNP_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNP_DIR}/install.bash"

  run dnp::update_bashrc_dnp_bin_path "${dnp_bin_dir}"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg called with args: Updating dnp entrypoint path in ~/.bashrc"

  # Should update ~/.bashrc
  assert_file_contains "${HOME}/.bashrc" "^export _DNP_PATH=\"${dnp_bin_dir}\"$"
}

@test "dnp::create_entrypoint_symlink_if_requested with option_system_wide_symlink=true › expect symlink created" {
  # Test case: When dnp::create_entrypoint_symlink_if_requested is called with option_system_wide_symlink=true, it should create a symlink
  local dnp_entrypoint="${TEMP_DNP_DIR}/src/bin/dnp"
  local option_system_wide_symlink=true
  local option_yes=true

  source "${TEMP_DNP_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNP_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNP_DIR}/install.bash"

  # Create the directory for the symlink
  mkdir -p "/usr/local/bin"
  assert_not_symlink_to "${dnp_entrypoint}" "/usr/local/bin/dnp"

  run dnp::create_entrypoint_symlink_if_requested "${option_system_wide_symlink}" "${option_yes}" "${dnp_entrypoint}"

  # Should succeed
  assert_success
  assert_symlink_to "${dnp_entrypoint}" "/usr/local/bin/dnp"

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg called with args: Creating symlink:"
}

@test "dnp::create_entrypoint_symlink_if_requested with option_system_wide_symlink=false › expect no symlink" {
  # Test case: When dnp::create_entrypoint_symlink_if_requested is called with option_system_wide_symlink=false, it should not create a symlink
  local dnp_entrypoint="${TEMP_DNP_DIR}/src/bin/dnp"
  local option_system_wide_symlink=false
  local option_yes=true

  source "${TEMP_DNP_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNP_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNP_DIR}/install.bash"

  run dnp::create_entrypoint_symlink_if_requested "${option_system_wide_symlink}" "${option_yes}" "${dnp_entrypoint}"

  # Should succeed
  assert_success
  assert_not_symlink_to "${dnp_entrypoint}" "/usr/local/bin/dnp"

  # Should not output the symlink message
  refute_output --partial "Mock n2st::print_msg called with args: Creating symlink:"
}

@test "dnp::add_dnp_entrypoint_path_to_bashrc_if_requested with option_add_dnp_path_to_bashrc=true › expect bashrc updated" {
  # Test case: When dnp::add_dnp_entrypoint_path_to_bashrc_if_requested is called with option_add_dnp_path_to_bashrc=true, it should update ~/.bashrc
  local option_add_dnp_path_to_bashrc=true
  local option_yes=true
  local dnp_bin_dir="${TEMP_DNP_DIR}/src/bin"

#  tail -n 10 "${HOME}/.bashrc"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNP_PATH=.*$"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNP_PATH=\"${dnp_bin_dir}\"$"
  assert_file_not_contains "${HOME}/.bashrc" "^export PATH=\"\$PATH:\$_DNP_PATH\"$"

  source "${TEMP_DNP_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNP_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNP_DIR}/install.bash"

  run dnp::add_dnp_entrypoint_path_to_bashrc_if_requested "${option_add_dnp_path_to_bashrc}" "${option_yes}" "${dnp_bin_dir}"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg called with args: Adding dnp entrypoint path to ~/.bashrc"

  # Should update ~/.bashrc
  assert_file_contains "${HOME}/.bashrc" "export _DNP_PATH=\"${dnp_bin_dir}\""
  assert_file_contains "${HOME}/.bashrc" "export PATH=\"\$PATH:\$_DNP_PATH\""
}

@test "dnp::add_dnp_entrypoint_path_to_bashrc_if_requested with option_add_dnp_path_to_bashrc=false › expect no bashrc update" {
  # Test case: When dnp::add_dnp_entrypoint_path_to_bashrc_if_requested is called with option_add_dnp_path_to_bashrc=false, it should not update ~/.bashrc
  local option_add_dnp_path_to_bashrc=false
  local option_yes=true
  local dnp_bin_dir="${TEMP_DNP_DIR}/src/bin"

  source "${TEMP_DNP_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNP_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNP_DIR}/install.bash"

  run dnp::add_dnp_entrypoint_path_to_bashrc_if_requested "${option_add_dnp_path_to_bashrc}" "${option_yes}" "${dnp_bin_dir}"

  # Should succeed
  assert_success
  assert_file_not_contains "${HOME}/.bashrc" "export _DNP_PATH=\"${dnp_bin_dir}\""
  assert_file_not_contains "${HOME}/.bashrc" "export PATH=\"\$PATH:\$_DNP_PATH\""

  # Should not output the bashrc update message
  refute_output --partial "Mock n2st::print_msg called with args: Adding dnp entrypoint path to ~/.bashrc"
  refute_output --partial "Mock n2st::print_msg called with args: Updating dnp entrypoint path in ~/.bashrc"
}
