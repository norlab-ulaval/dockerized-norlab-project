#!/usr/bin/env bats

bats_path=/usr/lib/bats
if [[ -d ${bats_path} ]]; then
  load "${bats_path}/bats-support/load"
  load "${bats_path}/bats-assert/load"
  load "${bats_path}/bats-file/load"
  load "${SRC_CODE_PATH:?err}/${N2ST_BATS_TESTING_TOOLS_RELATIVE_PATH:?err}/bats_helper_functions"
  load "${SRC_CODE_PATH}/tests/tests_bats/bats_testing_tools/bats_helper_functions_local"
else
  exit 1
fi

setup_file() {
  apt-get update && \
      apt-get install --yes rsync

  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  
  # Create a dummy template file
  mkdir -p "${BATS_DOCKER_WORKDIR}/src/lib/template"
  echo "template content" > "${BATS_DOCKER_WORKDIR}/src/lib/template/dummy_test_patch.txt"
}

setup() {
  export TEST_TEMP_DIR=$(temp_make)
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1
  
  export SUPER_PROJECT_ROOT="${TEST_TEMP_DIR}"
  export SUPER_PROJECT_REPO_NAME="test-project"
  
  # Mock dependencies that might be missing in test environment
  function n2st::seek_and_modify_string_in_file() {
    sed -i "s/$1/$2/g" "$3"
  }
  export -f n2st::seek_and_modify_string_in_file
}

teardown() {
  if [[ -n "${TEST_TEMP_DIR}" && -d "${TEST_TEMP_DIR}" ]]; then
    temp_del "${TEST_TEMP_DIR}"
  fi
}

teardown_file() {
  if [[ -n "${TEST_TEMP_DIR}" && -d "${TEST_TEMP_DIR}" ]]; then
    temp_del "${TEST_TEMP_DIR}"
  fi
  rm -f "${BATS_DOCKER_WORKDIR}/src/lib/template/dummy_test_patch.txt"
}

@test "dna::patch_check_and_run › should not run when versions match" {
  export DNA_CONFIG_SCHEME_VERSION=999
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=999
  
  run dna::patch_check_and_run
  assert_success
  assert_output ""
}

@test "dna::patch_check_and_run › should trigger patch when version is older" {
  export DNA_CONFIG_SCHEME_VERSION=-1
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=0
  
  # Setup mock super project
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  echo "DNA_CONFIG_SCHEME_VERSION=-1" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"
  
  # Mock user input to return 'y'
  function dna::patch_prompt_user() {
    REPLY="y"
  }
  export -f dna::patch_prompt_user

  run dna::patch_check_and_run
  
  assert_success
  assert_output --partial "Super project configuration scheme (v-1) is outdated"
  assert_output --partial "Applying configuration scheme patch: v-1 → v0"
  assert_output --partial "Dummy test patch file"
  assert_output --partial "Super project configuration scheme successfully updated to v0"
  
  assert_file_exist "${TEST_TEMP_DIR}/dummy_test_patch.txt"
  run grep "DNA_CONFIG_SCHEME_VERSION=0" "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"
  assert_success
}

@test "dna::patch_check_and_run › should handle skipping a resource" {
  export DNA_CONFIG_SCHEME_VERSION=-1
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=0
  
  # Setup mock super project
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  echo "DNA_CONFIG_SCHEME_VERSION=-1" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project-skip"
  export SUPER_PROJECT_REPO_NAME="test-project-skip"
  
  # Mock user input to return 'n'
  function dna::patch_prompt_user() {
    REPLY="n"
  }
  export -f dna::patch_prompt_user

  run dna::patch_check_and_run
  
  assert_success
  assert_output --partial "Skipping dummy_test_patch.txt"
  assert_output --partial "No changes were made to the super project structure"
  
  assert_file_not_exist "${TEST_TEMP_DIR}/dummy_test_patch.txt"
  # Version should still be updated as the patch script finished
  run grep "DNA_CONFIG_SCHEME_VERSION=0" "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project-skip"
  assert_success
}

@test "dna::patch_check_and_run › should handle incremental patches (e.g., 1002 to 1004)" {
  export DNA_CONFIG_SCHEME_VERSION=1002
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=1004
  
  # Setup mock super project
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  echo "DNA_CONFIG_SCHEME_VERSION=1002" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project-incremental"
  export SUPER_PROJECT_REPO_NAME="test-project-incremental"
  
  # Create mock patch scripts
  mkdir -p "${BATS_DOCKER_WORKDIR}/src/lib/core/patches"
  echo "echo 'Patching 1002 to 1003'" > "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_1002to1003.bash"
  echo "echo 'Patching 1003 to 1004'" > "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_1003to1004.bash"
  
  # Mock user input to return 'y'
  function dna::patch_prompt_user() {
    REPLY="y"
  }
  export -f dna::patch_prompt_user
  
  run dna::patch_check_and_run
  
  assert_success
  assert_output --partial "Applying configuration scheme patch: v1002 → v1003"
  assert_output --partial "Patching 1002 to 1003"
  assert_output --partial "Applying configuration scheme patch: v1003 → v1004"
  assert_output --partial "Patching 1003 to 1004"
  assert_output --partial "Super project configuration scheme successfully updated to v1004"
  
  run grep "DNA_CONFIG_SCHEME_VERSION=1004" "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project-incremental"
  assert_success
  
  # Cleanup mock patches
  rm "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_1002to1003.bash"
  rm "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_1003to1004.bash"
}

@test "dna::patch_check_and_run › should not update version if a patch fails" {
  export DNA_CONFIG_SCHEME_VERSION=1002
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=1004
  
  # Setup mock super project
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  echo "DNA_CONFIG_SCHEME_VERSION=1002" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project-fail"
  export SUPER_PROJECT_REPO_NAME="test-project-fail"
  
  # Create mock patch scripts: first succeeds, second fails
  mkdir -p "${BATS_DOCKER_WORKDIR}/src/lib/core/patches"
  echo "echo 'Patching 1002 to 1003'" > "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_1002to1003.bash"
  echo "echo 'Patching 1003 to 1004'; exit 1" > "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_1003to1004.bash"
  
  run dna::patch_check_and_run
  
  assert_failure
  # Version in file should still be 1002 because the whole process failed before finale stage
  run grep "DNA_CONFIG_SCHEME_VERSION=1002" "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project-fail"
  assert_success
  
  # Cleanup mock patches
  rm "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_1002to1003.bash"
  rm "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_1003to1004.bash"
}

@test "dna::patch_add_content_if_missing › should add content if search string is missing" {
  local test_file="existing_file.txt"
  echo "Initial content" > "${TEST_TEMP_DIR}/${test_file}"
  
  # Mock user input to return 'y'
  function dna::patch_prompt_user() {
    REPLY="y"
  }
  export -f dna::patch_prompt_user
  
  # Global array used by the function
  declare -a added_resources=()

  run dna::patch_add_content_if_missing "${test_file}" "New content" "New content line" "Dummy content"
  
  assert_success
  assert_output --partial "Missing Dummy content in existing_file.txt"
  run grep "New content line" "${TEST_TEMP_DIR}/${test_file}"
  assert_success
}

@test "dna::patch_add_content_if_missing › should not add content if search string exists" {
  local test_file="existing_file_with_content.txt"
  echo "Initial content" > "${TEST_TEMP_DIR}/${test_file}"
  echo "Existing search string" >> "${TEST_TEMP_DIR}/${test_file}"
  
  run dna::patch_add_content_if_missing "${test_file}" "Existing search string" "Should not be added" "Dummy content"
  
  assert_success
  assert_output ""
  run grep "Should not be added" "${TEST_TEMP_DIR}/${test_file}"
  assert_failure
}

@test "dna::patch_modify_content › should modify content if search pattern is found" {
  local test_file="file_to_modify.txt"
  echo "Initial content" > "${TEST_TEMP_DIR}/${test_file}"
  echo "OLD_VALUE=v1" >> "${TEST_TEMP_DIR}/${test_file}"
  
  # Mock user input to return 'y'
  function dna::patch_prompt_user() {
    REPLY="y"
  }
  export -f dna::patch_prompt_user
  
  # Global array used by the function
  declare -a added_resources=()

  run dna::patch_modify_content "${test_file}" "OLD_VALUE=v1" "OLD_VALUE=v2" "Update variable"
  
  assert_success
  assert_output --partial "Modifying Update variable in file_to_modify.txt"
  run grep "OLD_VALUE=v2" "${TEST_TEMP_DIR}/${test_file}"
  assert_success
  run grep "OLD_VALUE=v1" "${TEST_TEMP_DIR}/${test_file}"
  assert_failure
}

@test "dna::patch_modify_content › multi-line replacement should produce actual newlines (regression: no literal \\n)" {
  # Regression test for a bug where a multi-line replacement string such as
  #   'DNA_SJOB_NAME="$( ... )"\nexport DNA_SJOB_NAME'
  # was injected into the super project as a single line containing the literal
  # two-character sequence "\n" instead of an actual newline.
  local test_file="multiline_target.bash"
  printf '%s\n' '#!/bin/bash' 'DNA_SJOB_NAME="default"' 'echo ok' > "${TEST_TEMP_DIR}/${test_file}"

  function dna::patch_prompt_user() { REPLY="y"; }
  export -f dna::patch_prompt_user
  declare -a added_resources=()

  local search='DNA_SJOB_NAME="default"'
  # Replacement contains an actual newline character (as bash $'...' interprets \n).
  local replace=$'DNA_SJOB_NAME="$( basename "${BASH_SOURCE[0]}" | sed \'s/^slurm_job\\.//;s/\\.bash$//\' )"\nexport DNA_SJOB_NAME'

  run dna::patch_modify_content "${test_file}" "${search}" "${replace}" "Replace with multi-line assignment"
  assert_success

  # The resulting file must contain two SEPARATE lines (actual newline), not a
  # single line with a literal backslash-n.
  run grep -cF '\n' "${TEST_TEMP_DIR}/${test_file}"
  assert_output "0"

  run grep -cE '^export DNA_SJOB_NAME$' "${TEST_TEMP_DIR}/${test_file}"
  assert_output "1"

  run grep -cE '^DNA_SJOB_NAME="\$\( basename' "${TEST_TEMP_DIR}/${test_file}"
  assert_output "1"
}

@test "dna::patch_modify_content › patterns containing ';' should not be split by sed delimiter" {
  # Regression test: a `;` in the replacement must not corrupt the result.
  local test_file="semicolon_target.bash"
  printf '%s\n' 'REPLACE_ME' 'other line' > "${TEST_TEMP_DIR}/${test_file}"

  function dna::patch_prompt_user() { REPLY="y"; }
  export -f dna::patch_prompt_user
  declare -a added_resources=()

  local replace='sed '\''s/^a\.//;s/\.b$//'\'' done'

  run dna::patch_modify_content "${test_file}" "REPLACE_ME" "${replace}" "Semicolon-bearing replacement"
  assert_success

  run grep -cF "s/^a\\.//;s/\\.b\$//" "${TEST_TEMP_DIR}/${test_file}"
  assert_output "1"
}

@test "dna::_patch_fixed_string_replace_in_file › heredoc opener must not be chained with '|| return' (bash 3.2 re-parse regression)" {
  # Regression test for a bash 3.2 (macOS default /bin/bash) failure observed
  # when dna::_patch_fixed_string_replace_in_file was re-parsed in a child shell
  # via 'export -f':
  #   bash: dna::_patch_fixed_string_replace_in_file: line NN: syntax error near unexpected token `||'
  #   bash: error importing function definition for `dna::_patch_fixed_string_replace_in_file'
  # Root cause: bash 3.2's parser chokes on a heredoc opener chained with '||'
  # on the same line, e.g. `python3 - "$f" <<'PY' || return 1`. The fix is to
  # check $? on its own line after the heredoc terminator.
  local fn_file="${BATS_DOCKER_WORKDIR}/src/lib/core/utils/patch_helper.bash"
  # Must not contain the offending pattern anywhere in the file.
  run grep -E "<<'?[A-Za-z_][A-Za-z_0-9]*'?[[:space:]]+\\|\\|" "${fn_file}"
  assert_failure
}

@test "dna::patch_modify_content › should not modify content if search pattern is not found" {
  local test_file="file_not_to_modify.txt"
  echo "Initial content" > "${TEST_TEMP_DIR}/${test_file}"
  
  run dna::patch_modify_content "${test_file}" "NON_EXISTENT" "REPLACEMENT" "Should not run"
  
  assert_success
  assert_output ""
  run grep "REPLACEMENT" "${TEST_TEMP_DIR}/${test_file}"
  assert_failure
}

@test "dna::patch_prompt_user › should handle 'a' for yes to all" {
  local DNA_PATCH_YES_TO_ALL="false"
  
  DNA_PATCH_YES_TO_ALL="true"
  dna::patch_prompt_user "Dummy prompt"
  assert_equal "${REPLY}" "y"
}

@test "dna::patch_check_and_run › should handle 'yes to all' across multiple resources" {
  export DNA_CONFIG_SCHEME_VERSION=-1
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=0
  
  # Setup mock super project
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  echo "DNA_CONFIG_SCHEME_VERSION=-1" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project-all"
  export SUPER_PROJECT_REPO_NAME="test-project-all"
  
  # Create a patch that adds two things
  echo "dna::patch_add_file_if_missing 'file1.txt' 'file1.txt' 'File 1'" > "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_-1to0.bash"
  echo "dna::patch_add_file_if_missing 'file2.txt' 'file2.txt' 'File 2'" >> "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_-1to0.bash"
  
  echo "content 1" > "${BATS_DOCKER_WORKDIR}/src/lib/template/file1.txt"
  echo "content 2" > "${BATS_DOCKER_WORKDIR}/src/lib/template/file2.txt"

  # Mock dna::patch_prompt_user to simulate user entering 'a' on first call
  # Subsequent calls should not prompt if 'a' was selected.
  function dna::patch_prompt_user() {
    if [[ "${DNA_PATCH_YES_TO_ALL}" == "true" ]]; then
       REPLY="y"
       return
    fi
    # Simulate user typing 'a'
    DNA_PATCH_YES_TO_ALL="true"
    REPLY="y"
  }
  export -f dna::patch_prompt_user

  run dna::patch_check_and_run
  
  assert_success
  assert_file_exist "${TEST_TEMP_DIR}/file1.txt"
  assert_file_exist "${TEST_TEMP_DIR}/file2.txt"
  
  # Cleanup
  rm "${BATS_DOCKER_WORKDIR}/src/lib/template/file1.txt"
  rm "${BATS_DOCKER_WORKDIR}/src/lib/template/file2.txt"
  # Restore original test patch
  echo 'if [[ -f "${DNA_LIB_PATH}/template/dummy_test_patch.txt" ]]; then' > "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_-1to0.bash"
  echo '    dna::patch_add_file_if_missing "dummy_test_patch.txt" "dummy_test_patch.txt" "Dummy test patch file"' >> "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_-1to0.bash"
  echo 'fi' >> "${BATS_DOCKER_WORKDIR}/src/lib/core/patches/config_scheme_-1to0.bash"
}
