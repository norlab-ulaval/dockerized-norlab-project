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
}

setup() {
  export TEST_TEMP_DIR=$(temp_make)
  BATS_DOCKER_WORKDIR=$(pwd)
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1
  
  export SUPER_PROJECT_ROOT="${TEST_TEMP_DIR}"
  export SUPER_PROJECT_REPO_NAME="test-project"
  
  # Setup mock super project with v3
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  echo "DNA_CONFIG_SCHEME_VERSION=3" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"
  
  # Mock user input to return 'y'
  function dna::patch_prompt_user() {
    REPLY="y"
  }
  export -f dna::patch_prompt_user

  # Mock n2st functions
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

@test "config_scheme_3to4.bash › should add new v4 resources" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=4
  export DNA_CONFIG_SCHEME_VERSION=3
  
  run dna::patch_check_and_run
  
  assert_success
  assert_output --partial "Applying configuration scheme patch: v3 → v4"
  
  # Verify directories
  assert_dir_exist "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile"
  assert_dir_exist "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/overrides"
  
  # Verify some files in those directories
  assert_file_exist "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/.env.compute_canada"
  assert_file_exist "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/overrides/docker-compose.global.override.yaml"
  
  # Verify slurm job templates
  assert_file_exist "${TEST_TEMP_DIR}/slurm_jobs/slurm_job.apptainer.compute_canada.template.bash"
  assert_file_exist "${TEST_TEMP_DIR}/slurm_jobs/slurm_job.hydra.template.bash"
  
  # Verify version update
  run grep "DNA_CONFIG_SCHEME_VERSION=4" "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"
  assert_success
}
