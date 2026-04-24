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

  # Setup mock super project with v4
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  echo "DNA_CONFIG_SCHEME_VERSION=4" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"

  # Pre-create HPC server profile files with old v4 content (containing APPTAINER_CACHEDIR/TMPDIR)
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile"
  for hpc_profile_file in ".env.valeria" ".env.compute_canada" ".env.mamba"; do
    cat > "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/${hpc_profile_file}" << 'EOF'
# HPC server profile
DN_PROJECT_USER=testuser
# ....Apptainer cache configuration................................................................
# Apptainer cache and temp directories (use local scratch for performance)
APPTAINER_CACHEDIR="${HOME}/.apptainer/cache"
# Note: Set APPTAINER_TMPDIR to $SLURM_TMPDIR in your SBATCH script for best performance
APPTAINER_TMPDIR=/tmp
EOF
  done

  # Pre-create slurm job templates with old v4 content (containing 7-day time limit)
  mkdir -p "${TEST_TEMP_DIR}/slurm_jobs/template"
  for template in \
      "slurm_job.DNA_SJOB_NAME.bash" \
      "slurm_job.DNA_SJOB_NAME.hydra.bash" \
      "slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash" \
      "slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash" \
      "slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash" \
      "slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"; do
    cat > "${TEST_TEMP_DIR}/slurm_jobs/template/${template}" << 'EOF'
#!/bin/bash
#SBATCH --time=7-00:00
#SBATCH --cpus-per-task=4
# placeholder slurm job template for testing
python_arguments+=("launcher/example.py")
EOF
  done

  # Pre-create valeria template with old v4 content (includes Note line required for hydra patch)
  cat > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash" << 'EOF'
#!/bin/bash
#SBATCH --time=7-00:00
#SBATCH --cpus-per-task=4
function job_setup_callback() {
  # Add any instruction that should be executed before the apptainer exec command
  :
}
# ....Python module................................................................................
# TODO: Set python module to launch
python_arguments+=("launcher/example.py")
# Note: container workdir is <DN_PROJECT_PATH>/src/ (set in .env.valeria: DN_PROJECT_PATH)
# ....HPC server configuration.....................................................................
SUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"
# Set APPTAINER_TMPDIR to SLURM_TMPDIR for best performance on Valeria
APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"
export APPTAINER_TMPDIR
EOF

  # Pre-create compute_canada template with old v4 content (includes Note line required for hydra patch)
  cat > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash" << 'EOF'
#!/bin/bash
#SBATCH --time=7-00:00
#SBATCH --cpus-per-task=4
# ....Python module................................................................................
# TODO: Set python module to launch
python_arguments+=("launcher/example.py")
# Note: container workdir is <DN_PROJECT_PATH>/src/ (set in .env.compute_canada: DN_PROJECT_PATH)
# ....HPC server configuration.....................................................................
SUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"
EOF

  # Pre-create mamba template with old v4 content (includes Note line required for hydra patch)
  cat > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash" << 'EOF'
#!/bin/bash
#SBATCH --time=7-00:00
#SBATCH --cpus-per-task=4
# ....Python module................................................................................
# TODO: Set python module to launch
python_arguments+=("launcher/example.py")
# Note: container workdir is <DN_PROJECT_PATH>/src/ (set in .env.mamba: DN_PROJECT_PATH)
# ....HPC server configuration.....................................................................
SUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"
EOF

  # Mock user input to return 'y'
  function dna::patch_prompt_user() {
    REPLY="y"
  }
  export -f dna::patch_prompt_user

  # Mock n2st functions — use perl for multi-line / special-character patterns
  function n2st::seek_and_modify_string_in_file() {
    local search_pattern="$1"
    local replace_pattern="$2"
    local file_path="$3"
    perl -0777 -i -pe \
      'BEGIN{ $s=shift; $r=shift } s/\Q$s\E/$r/gs' \
      "${search_pattern}" "${replace_pattern}" \
      "${file_path}"
  }
  export -f n2st::seek_and_modify_string_in_file
}

teardown() {
  if [[ -n "${TEST_TEMP_DIR}" && -d "${TEST_TEMP_DIR}" ]]; then
    temp_del "${TEST_TEMP_DIR}"
  fi
}

# =================================================================================================
# Tests: v4 → v5 patch
# =================================================================================================

@test "config_scheme_4to5.bash › should apply patch and report v4 → v5" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success
  assert_output --partial "Applying configuration scheme patch: v4 → v5"
}

@test "config_scheme_4to5.bash › should update DNA_CONFIG_SCHEME_VERSION to 5" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  run grep "DNA_CONFIG_SCHEME_VERSION=5" "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"
  assert_success
}

@test "config_scheme_4to5.bash › should remove APPTAINER_CACHEDIR from all HPC profile files" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  for hpc_profile_file in ".env.valeria" ".env.compute_canada" ".env.mamba"; do
    target_file="${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/${hpc_profile_file}"
    assert_file_exist "${target_file}"
    run grep "APPTAINER_CACHEDIR=" "${target_file}"
    assert_failure  # APPTAINER_CACHEDIR should have been removed
  done
}

@test "config_scheme_4to5.bash › should remove APPTAINER_TMPDIR from all HPC profile files" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  for hpc_profile_file in ".env.valeria" ".env.compute_canada" ".env.mamba"; do
    target_file="${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/${hpc_profile_file}"
    assert_file_exist "${target_file}"
    run grep "APPTAINER_TMPDIR=" "${target_file}"
    assert_failure  # APPTAINER_TMPDIR should have been removed
  done
}

@test "config_scheme_4to5.bash › should remove Apptainer cache configuration comment from all HPC profile files" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  for hpc_profile_file in ".env.valeria" ".env.compute_canada" ".env.mamba"; do
    target_file="${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/${hpc_profile_file}"
    run grep "Apptainer cache configuration" "${target_file}"
    assert_failure  # Apptainer cache configuration section comment should be removed
  done
}

@test "config_scheme_4to5.bash › should leave no orphaned APPTAINER cache values in HPC profile files" {
  # Regression test: the old implementation used sed substring replacement which left orphaned
  # partial content (e.g. '/tmp' or '"${HOME}/.apptainer/cache"') as standalone lines.
  # The corrected implementation deletes entire matching lines, leaving no orphaned values.
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  for hpc_profile_file in ".env.valeria" ".env.compute_canada" ".env.mamba"; do
    target_file="${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/${hpc_profile_file}"
    assert_file_exist "${target_file}"
    # The orphaned value lines must not exist as standalone content after patching
    run grep '\.apptainer/cache' "${target_file}"
    assert_failure  # No orphaned APPTAINER_CACHEDIR value should remain
    run grep '^/tmp$' "${target_file}"
    assert_failure  # No orphaned APPTAINER_TMPDIR=/tmp value should remain
    # Also verify the section comment itself is fully gone (not partially mangled)
    run grep '^\.\.\.\.' "${target_file}"
    assert_failure  # No orphaned trailing dots from the section header should remain
  done
}

@test "config_scheme_4to5.bash › should skip HPC profile files that do not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  # Remove the HPC profile files to test skip logic
  rm -f "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria"
  rm -f "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/.env.compute_canada"
  rm -f "${TEST_TEMP_DIR}/.dockerized_norlab/configuration/hpc_server_profile/.env.mamba"

  run dna::patch_check_and_run

  assert_success
}

@test "config_scheme_4to5.bash › should update SBATCH --time from 7-00:00 to 0-24:00 in all slurm job templates" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  for template in \
      "slurm_job.DNA_SJOB_NAME.bash" \
      "slurm_job.DNA_SJOB_NAME.hydra.bash" \
      "slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash" \
      "slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash" \
      "slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash" \
      "slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"; do
    target_file="${TEST_TEMP_DIR}/slurm_jobs/template/${template}"
    assert_file_exist "${target_file}"
    run grep "#SBATCH --time=0-24:00" "${target_file}"
    assert_success  # New 24h time limit should be present
    run grep "#SBATCH --time=7-00:00" "${target_file}"
    assert_failure  # Old 7-day time limit should be gone
  done
}

@test "config_scheme_4to5.bash › should skip slurm templates that do not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  # Remove slurm templates to test skip logic
  rm -rf "${TEST_TEMP_DIR}/slurm_jobs"

  run dna::patch_check_and_run

  assert_success
}

@test "config_scheme_4to5.bash › should replace APPTAINER_TMPDIR=SLURM_TMPDIR with val-mktemp-dir in valeria template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  assert_file_exist "${target_file}"

  # Old SLURM_TMPDIR-based assignment should be gone
  run grep 'APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"' "${target_file}"
  assert_failure

  # New val-mktemp-dir based assignments should be present
  run grep 'val-mktemp-dir' "${target_file}"
  assert_success
}

@test "config_scheme_4to5.bash › should add APPTAINER_CACHEDIR val-mktemp-dir to valeria template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  run grep 'export APPTAINER_CACHEDIR="\$( val-mktemp-dir )"' "${target_file}"
  assert_success
}

@test "config_scheme_4to5.bash › should add APPTAINER_TMPDIR val-mktemp-dir export to valeria template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  run grep 'export APPTAINER_TMPDIR="\$( val-mktemp-dir )"' "${target_file}"
  assert_success
}

@test "config_scheme_4to5.bash › should add module load apptainer to job_setup_callback in valeria template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  run grep 'module load apptainer' "${target_file}"
  assert_success
}

@test "config_scheme_4to5.bash › should add module load httpproxy to job_setup_callback in valeria template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  run grep 'module load httpproxy' "${target_file}"
  assert_success
}

@test "config_scheme_4to5.bash › should add val-utils.sh source to job_setup_callback in valeria template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  run grep 'source /etc/profile.d/val-utils.sh' "${target_file}"
  assert_success
}

@test "config_scheme_4to5.bash › should skip valeria template patches if file does not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  rm -f "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  run dna::patch_check_and_run

  assert_success
}

@test "config_scheme_4to5.bash › should add optional hydra flags comment block to valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  run grep '# ....Optional hydra flags' "${target_file}"
  assert_success
}

@test "config_scheme_4to5.bash › should not duplicate optional hydra flags block in valeria template if already present" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  # Simulate already-patched file: replace the v4 Note line with the v5 content (Note + hydra block + HPC header)
  # dna::patch_modify_content uses n2st::seek_and_modify_string_in_file which is a no-op when the
  # search pattern is not found — so running the patch a second time on an already-patched file
  # won't add a duplicate.
  sed -i \
    's|# Note: container workdir is <DN_PROJECT_PATH>/src/ (set in .env.valeria: DN_PROJECT_PATH)|# Note: container workdir is <DN_PROJECT_PATH>/src/ (set in .env.valeria: DN_PROJECT_PATH)\n\n# ....Optional hydra flags.........................................................................\n#python_arguments+=("--config-path=")|' \
    "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  # Count occurrences — should appear exactly once
  count=$(grep -c '# ....Optional hydra flags' "${target_file}" || true)
  [ "${count}" -eq 1 ]
}

@test "config_scheme_4to5.bash › should add optional hydra flags comment block to compute_canada apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"

  run grep '# ....Optional hydra flags' "${target_file}"
  assert_success
}

@test "config_scheme_4to5.bash › should skip compute_canada template patches if file does not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  rm -f "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"

  run dna::patch_check_and_run

  assert_success
}

@test "config_scheme_4to5.bash › should add optional hydra flags comment block to mamba apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"

  run grep '# ....Optional hydra flags' "${target_file}"
  assert_success
}

@test "config_scheme_4to5.bash › should skip mamba template patches if file does not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=4

  rm -f "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"

  run dna::patch_check_and_run

  assert_success
}

@test "config_scheme_4to5.bash › should not apply patch when DNA_CONFIG_SCHEME_VERSION already equals DNA_RELEASE_CONFIG_SCHEME_VERSION" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=5
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success
  refute_output --partial "Applying configuration scheme patch: v4 → v5"
}
