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

  # Setup mock super project config dir with v5 version marker
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  echo "DNA_CONFIG_SCHEME_VERSION=5" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"

  # Create stub slurm job files (old/outdated content) that the patch will replace
  mkdir -p "${TEST_TEMP_DIR}/slurm_jobs/template"
  printf '# outdated stub\n' > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash"
  printf '# outdated stub\n' > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash"
  printf '# outdated stub\n' > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash"
  printf '# outdated stub\n' > "${TEST_TEMP_DIR}/slurm_jobs/slurm_job.dryrun.bash"

  # Apptainer stubs contain PLACEHOLDER so we can verify substitution
  printf '# outdated stub\nSIF_PATH="artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif"\n' \
    > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  printf '# outdated stub\nSIF_PATH="artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif"\n' \
    > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  printf '# outdated stub\nSIF_PATH="artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif"\n' \
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
# Tests: v5 → v6 patch
# =================================================================================================

@test "config_scheme_5to6.bash › should apply patch and report v5 → v6" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success
  assert_output --partial "Applying configuration scheme patch: v5 → v6"
}

@test "config_scheme_5to6.bash › should update DNA_CONFIG_SCHEME_VERSION to 6" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  run grep "DNA_CONFIG_SCHEME_VERSION=6" "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"
  assert_success
}

@test "config_scheme_5to6.bash › should replace all slurm templates with current DNA template versions" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  # Verify outdated stub content is gone and each file now matches the real DNA template
  for template in \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash" \
      "slurm_jobs/slurm_job.dryrun.bash"; do
    target_file="${TEST_TEMP_DIR}/${template}"
    assert_file_exist "${target_file}"
    run grep "# outdated stub" "${target_file}"
    assert_failure  # Stub marker must be gone
  done
}

@test "config_scheme_5to6.bash › should set #SBATCH --output to artifact/slurm_jobs_logs in all templates" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  for template in \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash" \
      "slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash" \
      "slurm_jobs/slurm_job.dryrun.bash"; do
    target_file="${TEST_TEMP_DIR}/${template}"
    run grep "#SBATCH --output=artifact/slurm_jobs_logs/%x-%j.out" "${target_file}"
    assert_success  # New artifact/slurm_jobs_logs path should be present
  done
}

@test "config_scheme_5to6.bash › should skip slurm templates that do not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  rm -rf "${TEST_TEMP_DIR}/slurm_jobs"

  run dna::patch_check_and_run

  assert_success
}

# -------------------------------------------------------------------------------------------------
# Non-apptainer template DNA_SJOB_NAME auto-set
# -------------------------------------------------------------------------------------------------

@test "config_scheme_5to6.bash › should set auto-set DNA_SJOB_NAME in base slurm template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash"
  run grep 'basename "${BASH_SOURCE\[0\]}"' "${target_file}"
  assert_success  # auto-set line should be present
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure  # old hardcoded default should be gone
}

@test "config_scheme_5to6.bash › should set auto-set DNA_SJOB_NAME in hydra template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash"
  run grep "hydra\.bash" "${target_file}"
  assert_success  # profile-specific sed stripping .hydra.bash should be present
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should set auto-set DNA_SJOB_NAME in hydra_hparam_optim template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash"
  run grep "hydra_hparam_optim\.bash" "${target_file}"
  assert_success  # profile-specific sed stripping .hydra_hparam_optim.bash should be present
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should set auto-set DNA_SJOB_NAME in dryrun template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/slurm_job.dryrun.bash"
  run grep 'basename "${BASH_SOURCE\[0\]}"' "${target_file}"
  assert_success  # auto-set line should be present
  run grep 'DNA_SJOB_NAME="dryrun"' "${target_file}"
  assert_failure  # old hardcoded value should be gone
}

# -------------------------------------------------------------------------------------------------
# Valeria apptainer template
# -------------------------------------------------------------------------------------------------

@test "config_scheme_5to6.bash › should set auto-set DNA_SJOB_NAME in valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep "apptainer\.valeria\.bash" "${target_file}"
  assert_success  # profile-specific sed stripping .apptainer.valeria.bash should be present
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should add module spider version-aware load to valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep "module spider apptainer" "${target_file}"
  assert_success
}

@test "config_scheme_5to6.bash › should add APPTAINER_CACHEDIR mktemp to valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep 'export APPTAINER_CACHEDIR="\$( mktemp -d' "${target_file}"
  assert_success
}

@test "config_scheme_5to6.bash › should add APPTAINER_TMPDIR mktemp to valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep 'export APPTAINER_TMPDIR="\$( mktemp -d' "${target_file}"
  assert_success
}

@test "config_scheme_5to6.bash › should remove val-mktemp-dir from valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep "val-mktemp-dir" "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should remove val-utils.sh from valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep "val-utils.sh" "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should keep module load httpproxy in valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep "module load httpproxy" "${target_file}"
  assert_success
}

@test "config_scheme_5to6.bash › should substitute PLACEHOLDER_DN_PROJECT_IMAGE_NAME in valeria template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  # PLACEHOLDER_DN_PROJECT_IMAGE_NAME should be replaced with lowercased project name
  run grep "test-project" "${target_file}"
  assert_success  # substituted image name should be present
  run grep "PLACEHOLDER_DN_PROJECT_IMAGE_NAME" "${target_file}"
  assert_failure  # placeholder must be gone
}

@test "config_scheme_5to6.bash › should skip valeria template patches if file does not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  rm -f "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  run dna::patch_check_and_run

  assert_success
}

# -------------------------------------------------------------------------------------------------
# Compute Canada apptainer template
# -------------------------------------------------------------------------------------------------

@test "config_scheme_5to6.bash › should set auto-set DNA_SJOB_NAME in compute_canada apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  run grep "apptainer\.compute_canada\.bash" "${target_file}"
  assert_success
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should replace APPTAINER_TMPDIR=SLURM_TMPDIR with mktemp in compute_canada template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"

  run grep 'APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"' "${target_file}"
  assert_failure  # Old assignment should be gone

  run grep 'export APPTAINER_CACHEDIR="\$( mktemp -d' "${target_file}"
  assert_success  # New mktemp-based CACHEDIR should be present
  run grep 'export APPTAINER_TMPDIR="\$( mktemp -d' "${target_file}"
  assert_success  # New mktemp-based TMPDIR should be present
}

@test "config_scheme_5to6.bash › should add module spider version-aware load to compute_canada apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  run grep "module spider apptainer" "${target_file}"
  assert_success
}

@test "config_scheme_5to6.bash › should substitute PLACEHOLDER_DN_PROJECT_IMAGE_NAME in compute_canada template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  run grep "test-project" "${target_file}"
  assert_success
  run grep "PLACEHOLDER_DN_PROJECT_IMAGE_NAME" "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should skip compute_canada template patches if file does not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  rm -f "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"

  run dna::patch_check_and_run

  assert_success
}

# -------------------------------------------------------------------------------------------------
# Mamba apptainer template
# -------------------------------------------------------------------------------------------------

@test "config_scheme_5to6.bash › should set auto-set DNA_SJOB_NAME in mamba apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
  run grep "apptainer\.mamba\.bash" "${target_file}"
  assert_success
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should replace APPTAINER_TMPDIR=SLURM_TMPDIR with mktemp in mamba template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"

  run grep 'APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"' "${target_file}"
  assert_failure  # Old assignment should be gone

  run grep 'export APPTAINER_CACHEDIR="\$( mktemp -d' "${target_file}"
  assert_success
  run grep 'export APPTAINER_TMPDIR="\$( mktemp -d' "${target_file}"
  assert_success
}

@test "config_scheme_5to6.bash › should add module spider version-aware load to mamba apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
  run grep "module spider apptainer" "${target_file}"
  assert_success
}

@test "config_scheme_5to6.bash › should substitute PLACEHOLDER_DN_PROJECT_IMAGE_NAME in mamba template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
  run grep "test-project" "${target_file}"
  assert_success
  run grep "PLACEHOLDER_DN_PROJECT_IMAGE_NAME" "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should skip mamba template patches if file does not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  rm -f "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"

  run dna::patch_check_and_run

  assert_success
}

# -------------------------------------------------------------------------------------------------
# Idempotency / version guard
# -------------------------------------------------------------------------------------------------

@test "config_scheme_5to6.bash › should not apply patch when DNA_CONFIG_SCHEME_VERSION already equals DNA_RELEASE_CONFIG_SCHEME_VERSION" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=6

  run dna::patch_check_and_run

  assert_success
  refute_output --partial "Applying configuration scheme patch: v5 → v6"
}
