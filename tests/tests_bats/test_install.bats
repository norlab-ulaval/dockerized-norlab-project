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
  export TEMP_DNA_DIR="${TEMP_DIR}/$(basename "${BATS_DOCKER_WORKDIR}")"
  cp -R "${BATS_DOCKER_WORKDIR}" "${TEMP_DNA_DIR}/"

#  tree -L 4 -a "$BATS_DOCKER_WORKDIR/src"  >&3

  # Create a mock import_norlab_shell_script_tools_lib.bash
  cat > "${TEMP_DNA_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash" << 'EOF'
#!/bin/bash
# Mock import_norlab_shell_script_tools_lib.bash

tmp_pwd=$(pwd)
cd ${TEMP_DNA_DIR}/utilities/norlab-shell-script-tools/src/function_library/
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
  cat > "${TEMP_DNA_DIR}/src/lib/core/utils/ui.bash" << 'EOF'
#!/bin/bash
# Mock ui.bash

function dna::command_help_menu() {
  echo "Mock dna::command_help_menu called with args: $*"
  return 0
}

function dna::unknown_option_msg() {
  echo "Mock dna::unknown_option_msg called with args: $*"
  return 1
}

# Export mock functions
for func in $(compgen -A function | grep -e dna::); do
  # shellcheck disable=SC2163
  export -f "${func}"
done

# Print a message to indicate that the mock ui.bash has been loaded
echo "[DNA done] Mock ui.bash loaded"
EOF

  # Create a mock setup_host_dna_requirements.bash
  cat > "${TEMP_DNA_DIR}/src/lib/core/utils/setup_host_dna_requirements.bash" << 'EOF'
#!/bin/bash
# Mock setup_host_dna_requirements.bash

echo "Mock setup_host_dna_requirements.bash executed"
exit 0
EOF
}

setup() {
  cd "${TEMP_DNA_DIR}" || exit 1
}

# ....Teardown.....................................................................................
teardown() {
  bats_print_run_env_variable_on_error
  rm -f /usr/local/bin/dna
  sed -i '/# >>>> Dockerized-NorLab Project (start)/,/# <<<< Dockerized-NorLab Project (end)/d' "${HOME}/.bashrc"
  cd "${TEMP_DNA_DIR}" || exit 1
}

teardown_file() {
  # Clean up temporary directories
  temp_del "${TEMP_DIR}"
  #tree -L 2 -a $PWD >&3
}

# ====Test cases==================================================================================

# Test case for help option
@test "dna::install_dockerized_norlab_project_on_host with --help › expect help menu" {
  # Test case: When install.bash is called with --help, it should show the help menu
  run bash -c "source ${TEMP_DNA_DIR}/install.bash && dna::install_dockerized_norlab_project_on_host --help"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::install_dockerized_norlab_project_on_host with -h › expect help menu" {
  # Test case: When install.bash is called with -h, it should show the help menu
  run bash -c "source ${TEMP_DNA_DIR}/install.bash && dna::install_dockerized_norlab_project_on_host -h"

  # Should succeed
  assert_success

  # Should output the help menu
  assert_output --partial "Mock dna::command_help_menu called with args:"
}

@test "dna::install_dockerized_norlab_project_on_host with unknown option › expect error" {
  # Test case: When install.bash is called with an unknown option, it should show an error
  run bash -c "source ${TEMP_DNA_DIR}/install.bash && dna::install_dockerized_norlab_project_on_host --unknown-option"

  # Should fail
  assert_failure

  # Should output the error message
  assert_output --partial "Mock dna::unknown_option_msg called with args: bash ./install.bash"
}

@test "dna::install_dockerized_norlab_project_on_host with default options › expect system-wide symlink" {
  # Test case: When install.bash is called with default options, it should create a system-wide symlink
  run bash -c "source ${TEMP_DNA_DIR}/install.bash && dna::install_dockerized_norlab_project_on_host"

  # Should succeed
  assert_success

  assert_file_executable "${TEMP_DNA_DIR}/src/bin/dna"
  assert_symlink_to "${TEMP_DNA_DIR}/src/bin/dna" "/usr/local/bin/dna"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNA_PATH=.*$"

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Setting up host for Dockerized-NorLab Project"
  assert_output --partial "Mock n2st::print_msg called with args: Creating symlink:"
  assert_output --partial "Mock n2st::print_msg_done called with args: Dockerized-NorLab Project has been installed successfully"
  assert_output --partial "Mock n2st::print_msg called with args: You can now use 'dna' command from anywhere"

}


@test "dna::install_dockerized_norlab_project_on_host with --skip-system-wide-symlink-install › expect no symlink" {
  # Test case: When install.bash is called with --skip-system-wide-symlink-install, it should not create a symlink
  run bash -c "source ${TEMP_DNA_DIR}/install.bash && dna::install_dockerized_norlab_project_on_host --skip-system-wide-symlink-install"

  # Should succeed
  assert_success

  # Should not create a symlink
  assert_file_executable "${TEMP_DNA_DIR}/src/bin/dna"
  assert_not_symlink_to "${TEMP_DNA_DIR}/src/bin/dna" "/usr/local/bin/dna"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNA_PATH=.*$"
  refute_output --partial "Mock n2st::print_msg called with args: Creating symlink:"

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Setting up host for Dockerized-NorLab Project"
  assert_output --partial "Mock n2st::print_msg_done called with args: Dockerized-NorLab Project has been installed successfully"
  assert_output --partial "Mock n2st::print_msg called with args: You can use"
}

@test "dna::install_dockerized_norlab_project_on_host with --add-dna-path-to-bashrc › expect bashrc update" {
  # Test case: When install.bash is called with --add-dna-path-to-bashrc, it should update ~/.bashrc
  run bash -c "source ${TEMP_DNA_DIR}/install.bash && dna::install_dockerized_norlab_project_on_host --add-dna-path-to-bashrc"

  # Should succeed
  assert_success

  # Should not create a symlink
  assert_file_executable "${TEMP_DNA_DIR}/src/bin/dna"
  assert_not_symlink_to "${TEMP_DNA_DIR}/src/bin/dna" "/usr/local/bin/dna"
  assert_file_contains "${HOME}/.bashrc" "^export _DNA_PATH=.*$"
  refute_output --partial "Mock n2st::print_msg called with args: Creating symlink:"

  # Should update ~/.bashrc
  assert_output --partial "Mock n2st::print_msg called with args: Adding dna entrypoint path to ~/.bashrc"

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Setting up host for Dockerized-NorLab Project"
  assert_output --partial "Mock n2st::print_msg_done called with args: Dockerized-NorLab Project has been installed successfully"
  assert_output --partial "Mock n2st::print_msg called with args: After restarting your shell or sourcing ~/.bashrc"
}

@test "dna::install_dockerized_norlab_project_on_host with --yes › expect non-interactive installation" {
  # Test case: When install.bash is called with --yes, it should perform a non-interactive installation
  run bash -c "source ${TEMP_DNA_DIR}/install.bash && dna::install_dockerized_norlab_project_on_host --yes"

  # Should succeed
  assert_success

  assert_file_executable "${TEMP_DNA_DIR}/src/bin/dna"
  assert_symlink_to "${TEMP_DNA_DIR}/src/bin/dna" "/usr/local/bin/dna"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNA_PATH=.*$"

  # Should output the expected messages
  assert_output --partial "Mock n2st::print_msg called with args: Setting up host for Dockerized-NorLab Project"
  assert_output --partial "Mock n2st::print_msg called with args: Creating symlink:"
  assert_output --partial "Mock n2st::print_msg_done called with args: Dockerized-NorLab Project has been installed successfully"
  assert_output --partial "Mock n2st::print_msg called with args: You can now use 'dna' command from anywhere"
}

@test "dna::create_bin_dna_to_entrypoint_symlink › expect symlink created" {
  # Test case: When dna::create_bin_dna_to_entrypoint_symlink is called, it should create a symlink
  local dna_entrypoint="${TEMP_DNA_DIR}/src/bin/dna"
  local bin_dna_path="${TEMP_DIR}/usr/local/bin/dna"

  # Create the directory for the symlink
  mkdir -p "$(dirname "${bin_dna_path}")"

  source "${TEMP_DNA_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNA_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNA_DIR}/install.bash"

  assert_not_symlink_to "${dna_entrypoint}" "${bin_dna_path}"

  run dna::create_bin_dna_to_entrypoint_symlink "${dna_entrypoint}" "${bin_dna_path}"

  # Should succeed
  assert_success
  assert_symlink_to "${dna_entrypoint}" "${bin_dna_path}"

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg called with args: Creating symlink:"

}

@test "dna::update_bashrc_dna_bin_path › expect bashrc updated" {
  # Test case: When dna::update_bashrc_dna_bin_path is called, it should update ~/.bashrc
  local dna_bin_dir="${TEMP_DNA_DIR}/src/bin"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNA_PATH=.*$"

  # Create a ~/.bashrc file with existing DNA_PATH entries
  cat > "${HOME}/.bashrc" << EOF
# Existing .bashrc content
# >>>> Dockerized-NorLab Project (start)
export _DNA_PATH="/old/path"
export PATH="\$PATH:\$_DNA_PATH"
# <<<< Dockerized-NorLab Project (end)
EOF

  source "${TEMP_DNA_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNA_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNA_DIR}/install.bash"

  run dna::update_bashrc_dna_bin_path "${dna_bin_dir}"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg called with args: Updating dna entrypoint path in ~/.bashrc"

  # Should update ~/.bashrc
  assert_file_contains "${HOME}/.bashrc" "^export _DNA_PATH=\"${dna_bin_dir}\"$"
}

@test "dna::create_entrypoint_symlink_if_requested with option_system_wide_symlink=true › expect symlink created" {
  # Test case: When dna::create_entrypoint_symlink_if_requested is called with option_system_wide_symlink=true, it should create a symlink
  local dna_entrypoint="${TEMP_DNA_DIR}/src/bin/dna"
  local option_system_wide_symlink=true
  local option_yes=true

  source "${TEMP_DNA_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNA_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNA_DIR}/install.bash"

  # Create the directory for the symlink
  mkdir -p "/usr/local/bin"
  assert_not_symlink_to "${dna_entrypoint}" "/usr/local/bin/dna"

  run dna::create_entrypoint_symlink_if_requested "${option_system_wide_symlink}" "${option_yes}" "${dna_entrypoint}"

  # Should succeed
  assert_success
  assert_symlink_to "${dna_entrypoint}" "/usr/local/bin/dna"

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg called with args: Creating symlink:"
}

@test "dna::create_entrypoint_symlink_if_requested with option_system_wide_symlink=false › expect no symlink" {
  # Test case: When dna::create_entrypoint_symlink_if_requested is called with option_system_wide_symlink=false, it should not create a symlink
  local dna_entrypoint="${TEMP_DNA_DIR}/src/bin/dna"
  local option_system_wide_symlink=false
  local option_yes=true

  source "${TEMP_DNA_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNA_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNA_DIR}/install.bash"

  run dna::create_entrypoint_symlink_if_requested "${option_system_wide_symlink}" "${option_yes}" "${dna_entrypoint}"

  # Should succeed
  assert_success
  assert_not_symlink_to "${dna_entrypoint}" "/usr/local/bin/dna"

  # Should not output the symlink message
  refute_output --partial "Mock n2st::print_msg called with args: Creating symlink:"
}

@test "dna::add_dna_entrypoint_path_to_bashrc_if_requested with option_add_dna_path_to_bashrc=true › expect bashrc updated" {
  # Test case: When dna::add_dna_entrypoint_path_to_bashrc_if_requested is called with option_add_dna_path_to_bashrc=true, it should update ~/.bashrc
  local option_add_dna_path_to_bashrc=true
  local option_yes=true
  local dna_bin_dir="${TEMP_DNA_DIR}/src/bin"

#  tail -n 10 "${HOME}/.bashrc"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNA_PATH=.*$"
  assert_file_not_contains "${HOME}/.bashrc" "^export _DNA_PATH=\"${dna_bin_dir}\"$"
  assert_file_not_contains "${HOME}/.bashrc" "^export PATH=\"\$PATH:\$_DNA_PATH\"$"

  source "${TEMP_DNA_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNA_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNA_DIR}/install.bash"

  run dna::add_dna_entrypoint_path_to_bashrc_if_requested "${option_add_dna_path_to_bashrc}" "${option_yes}" "${dna_bin_dir}"

  # Should succeed
  assert_success

  # Should output the expected message
  assert_output --partial "Mock n2st::print_msg called with args: Adding dna entrypoint path to ~/.bashrc"

  # Should update ~/.bashrc
  assert_file_contains "${HOME}/.bashrc" "export _DNA_PATH=\"${dna_bin_dir}\""
  assert_file_contains "${HOME}/.bashrc" "export PATH=\"\$PATH:\$_DNA_PATH\""
}

@test "dna::add_dna_entrypoint_path_to_bashrc_if_requested with option_add_dna_path_to_bashrc=false › expect no bashrc update" {
  # Test case: When dna::add_dna_entrypoint_path_to_bashrc_if_requested is called with option_add_dna_path_to_bashrc=false, it should not update ~/.bashrc
  local option_add_dna_path_to_bashrc=false
  local option_yes=true
  local dna_bin_dir="${TEMP_DNA_DIR}/src/bin"

  source "${TEMP_DNA_DIR}/load_repo_main_dotenv.bash"
  source "${TEMP_DNA_DIR}/utilities/norlab-shell-script-tools/import_norlab_shell_script_tools_lib.bash"
  source "${TEMP_DNA_DIR}/install.bash"

  run dna::add_dna_entrypoint_path_to_bashrc_if_requested "${option_add_dna_path_to_bashrc}" "${option_yes}" "${dna_bin_dir}"

  # Should succeed
  assert_success
  assert_file_not_contains "${HOME}/.bashrc" "export _DNA_PATH=\"${dna_bin_dir}\""
  assert_file_not_contains "${HOME}/.bashrc" "export PATH=\"\$PATH:\$_DNA_PATH\""

  # Should not output the bashrc update message
  refute_output --partial "Mock n2st::print_msg called with args: Adding dna entrypoint path to ~/.bashrc"
  refute_output --partial "Mock n2st::print_msg called with args: Updating dna entrypoint path in ~/.bashrc"
}
