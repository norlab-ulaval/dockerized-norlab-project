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
  
  # Verify slurm job templates directory and files
  assert_dir_exist "${TEST_TEMP_DIR}/slurm_jobs/template"
  assert_file_exist "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  assert_file_exist "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.dna.bash"
  
  # Verify version update
  run grep "DNA_CONFIG_SCHEME_VERSION=4" "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"
  assert_success
}

@test "config_scheme_3to4.bash › should replace PLACEHOLDER_DN_PROJECT_GIT_NAME in HPC server profile files" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=4
  export DNA_CONFIG_SCHEME_VERSION=3

  run dna::patch_check_and_run

  assert_success

  # Verify placeholder was replaced in all HPC server profile files
  for hpc_profile_file in ".env.valeria" ".env.compute_canada" ".env.mamba"; do
    target_file="${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/${hpc_profile_file}"
    assert_file_exist "${target_file}"
    run grep "PLACEHOLDER_DN_PROJECT_GIT_NAME" "${target_file}"
    assert_failure  # placeholder should NOT be present
    run grep "DN_PROJECT_PATH=/ros2_ws/src/test-project" "${target_file}"
    assert_success  # actual project name should be present
  done
}

@test "config_scheme_3to4.bash › should update slurm_job.dryrun.bash with new content when pre-existing" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=4
  export DNA_CONFIG_SCHEME_VERSION=3

  # Pre-create slurm_job.dryrun.bash with old v3 content
  mkdir -p "${TEST_TEMP_DIR}/slurm_jobs"
  cat > "${TEST_TEMP_DIR}/slurm_jobs/slurm_job.dryrun.bash" << 'EOF'
#!/bin/bash
function dna::job_teardown_callback() {
  local exit_code=$?
  # Note: Command 'dna run slurm' already handle stoping the container in case the slurm command
  # TODO: Add any instruction that should be executed after 'dna run slurm' exit.
  #  `scancel` is issued.
  exit ${exit_code:-1}
}
# TODO: Set DNA_SJOB_NAME
DNA_SJOB_NAME="default"
# TODO: Set python module to launch
hydra_flags+=("launcher/example_app_hparm_optim.py")
EOF

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/slurm_job.dryrun.bash"
  assert_file_exist "${target_file}"

  # DNA_SJOB_NAME should be updated to 'dryrun'
  run grep 'DNA_SJOB_NAME="dryrun"' "${target_file}"
  assert_success

  # Old DNA_SJOB_NAME="default" should be gone
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure

  # TODO comments should be removed
  run grep '# TODO:' "${target_file}"
  assert_failure
  # Note: config_scheme_3to4 patch operates on the old slurm_job.dryrun.bash naming (pre-v6).
  # The rename to slurm_job.dryrun.dna.bash is handled by config_scheme_5to6.
}

@test "config_scheme_3to4.bash › should replace PLACEHOLDER_DN_PROJECT_GIT_NAME in pre-existing HPC server profile files" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=4
  export DNA_CONFIG_SCHEME_VERSION=3

  # Pre-create HPC profile files with placeholder (simulating files already present before patch)
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile"
  for hpc_profile_file in ".env.valeria" ".env.compute_canada" ".env.mamba"; do
    echo "DN_PROJECT_PATH=/ros2_ws/src/PLACEHOLDER_DN_PROJECT_GIT_NAME" \
      > "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/${hpc_profile_file}"
  done

  run dna::patch_check_and_run

  assert_success

  # Verify placeholder was replaced in all pre-existing HPC server profile files
  for hpc_profile_file in ".env.valeria" ".env.compute_canada" ".env.mamba"; do
    target_file="${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/${hpc_profile_file}"
    run grep "PLACEHOLDER_DN_PROJECT_GIT_NAME" "${target_file}"
    assert_failure  # placeholder should NOT be present
    run grep "DN_PROJECT_PATH=/ros2_ws/src/test-project" "${target_file}"
    assert_success  # actual project name should be present
  done
}

@test "config_scheme_3to4.bash › should replace PLACEHOLDER_DN_PROJECT_IMAGE_NAME with lowercased project name in slurm templates" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=4
  export DNA_CONFIG_SCHEME_VERSION=3
  # Use a mixed-case repo name to verify lowercasing
  export SUPER_PROJECT_REPO_NAME="Test-Project"
  # Create the meta dotenv file matching the mixed-case repo name (patch_helper looks for .env.${SUPER_PROJECT_REPO_NAME})
  echo "DNA_CONFIG_SCHEME_VERSION=3" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.Test-Project"

  run dna::patch_check_and_run

  assert_success

  # Verify placeholder is replaced and value is lowercase (matching DN_PROJECT_IMAGE_NAME convention)
  for template_file in \
    "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash" \
    "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash" \
    "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"; do
    target_file="${TEST_TEMP_DIR}/${template_file}"
    assert_file_exist "${target_file}"
    run grep "PLACEHOLDER_DN_PROJECT_IMAGE_NAME" "${target_file}"
    assert_failure  # placeholder should NOT be present
    # Note: SIF filenames are now fully versioned/target-aware (<image>-slurm-<version>-<target>.sif),
    # so assert the lowercased image name followed by the -slurm- prefix rather than the old -slurm.sif.
    run grep "test-project-slurm-" "${target_file}"
    assert_success  # lowercase image name with -slurm- prefix should be present
    run grep "Test-Project" "${target_file}"
    assert_failure  # mixed-case original should NOT be present
  done
}

@test "config_scheme_3to4.bash › should add DN_CONTAINER_NAME to pre-existing HPC server profile files" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=4
  export DNA_CONFIG_SCHEME_VERSION=3

  # Pre-create HPC profile files WITHOUT DN_CONTAINER_NAME (simulating v3 files before patch)
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile"
  for hpc_profile_file in ".env.valeria" ".env.compute_canada" ".env.mamba"; do
    echo "DN_PROJECT_PATH=/ros2_ws/src/PLACEHOLDER_DN_PROJECT_GIT_NAME" \
      > "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/${hpc_profile_file}"
  done

  run dna::patch_check_and_run

  assert_success

  # Verify DN_CONTAINER_NAME was added to all pre-existing HPC server profile files
  for hpc_profile_file in ".env.valeria" ".env.compute_canada" ".env.mamba"; do
    target_file="${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/${hpc_profile_file}"
    run grep "DN_CONTAINER_NAME=" "${target_file}"
    assert_success  # DN_CONTAINER_NAME should now be present
    run grep "PLACEHOLDER_DN_CONTAINER_NAME" "${target_file}"
    assert_failure  # placeholder should NOT be present
  done
}
