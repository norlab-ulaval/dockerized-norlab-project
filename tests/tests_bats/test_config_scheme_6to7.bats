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

  # Setup mock super project config dir with v6 version marker
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  echo "DNA_CONFIG_SCHEME_VERSION=6" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"

  # Apptainer stubs contain PLACEHOLDER + old (v6) SIF_PATH so we can verify substitution + replacement
  mkdir -p "${TEST_TEMP_DIR}/slurm_jobs/template"
  printf '# outdated stub\nSIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"\n' \
    > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  printf '# outdated stub\nSIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"\n' \
    > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  printf '# outdated stub\nSIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"\n' \
    > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"

  # Mock user input to return 'y' (avoids interactive read -r -n 1 prompt in non-interactive bats)
  function dna::patch_prompt_user() {
    REPLY="y"
  }
  export -f dna::patch_prompt_user
}

teardown() {
  if [[ -n "${TEST_TEMP_DIR}" && -d "${TEST_TEMP_DIR}" ]]; then
    temp_del "${TEST_TEMP_DIR}"
  fi
}

# =================================================================================================
# Tests: v6 → v7 patch
# =================================================================================================

@test "config_scheme_6to7.bash › should apply patch and report v6 → v7" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=7
  export DNA_CONFIG_SCHEME_VERSION=6

  run dna::patch_check_and_run

  assert_success
  assert_output --partial "Applying configuration scheme patch: v6 → v7"
}

@test "config_scheme_6to7.bash › should update DNA_CONFIG_SCHEME_VERSION to 7" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=7
  export DNA_CONFIG_SCHEME_VERSION=6

  run dna::patch_check_and_run

  assert_success

  run grep "DNA_CONFIG_SCHEME_VERSION=7" "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"
  assert_success
}

@test "config_scheme_6to7.bash › should replace apptainer templates with current DNA versions" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=7
  export DNA_CONFIG_SCHEME_VERSION=6

  run dna::patch_check_and_run

  assert_success

  for template in \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"; do
    target_file="${TEST_TEMP_DIR}/${template}"
    assert_file_exist "${target_file}"
    run grep "# outdated stub" "${target_file}"
    assert_failure  # Stub marker must be gone
  done
}

@test "config_scheme_6to7.bash › should resolve versioned SIF from \${SCRATCH}/sif with target suffix" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=7
  export DNA_CONFIG_SCHEME_VERSION=6

  run dna::patch_check_and_run
  assert_success

  # Each per-target template must resolve the newest versioned SIF at runtime via a glob:
  #   ${SCRATCH}/sif/<image>-slurm-*-<target>.sif
  run grep -F 'SIF_PATH="${SIF_PATH:-$(ls -t ${SCRATCH}/sif/test-project-slurm-*-valeria.sif 2>/dev/null | head -n1)}"' \
    "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  assert_success

  run grep -F 'SIF_PATH="${SIF_PATH:-$(ls -t ${SCRATCH}/sif/test-project-slurm-*-compute-canada.sif 2>/dev/null | head -n1)}"' \
    "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  assert_success

  run grep -F 'SIF_PATH="${SIF_PATH:-$(ls -t ${SCRATCH}/sif/test-project-slurm-*-mamba.sif 2>/dev/null | head -n1)}"' \
    "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
  assert_success
}

@test "config_scheme_6to7.bash › should substitute PLACEHOLDER_DN_PROJECT_IMAGE_NAME in templates" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=7
  export DNA_CONFIG_SCHEME_VERSION=6

  run dna::patch_check_and_run
  assert_success

  for template in \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"; do
    target_file="${TEST_TEMP_DIR}/${template}"
    run grep "PLACEHOLDER_DN_PROJECT_IMAGE_NAME" "${target_file}"
    assert_failure  # placeholder must be substituted
  done
}
