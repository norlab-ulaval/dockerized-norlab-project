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

  # Setup mock super project with v5
  mkdir -p "${TEST_TEMP_DIR}/.dockerized_norlab"
  echo "DNA_CONFIG_SCHEME_VERSION=5" > "${TEST_TEMP_DIR}/.dockerized_norlab/.env.test-project"

  # Pre-create slurm job templates with v5 content (output path 'out/', old DNA_SJOB_NAME="default")
  mkdir -p "${TEST_TEMP_DIR}/slurm_jobs/template"

  # Base template (v5 state)
  cat > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash" << 'EOF'
#!/bin/bash
#SBATCH --time=0-24:00
#SBATCH --output=out/%x-%j.out

declare -x DNA_SJOB_NAME
declare -a dna_run_slurm_flags=()
declare -a python_arguments=()
# ====Setup========================================================================================
# ....Custom setup (optional)......................................................................
function dna::job_setup_callback() {
  :
}
# ....Custom teardown (optional)...................................................................
function dna::job_teardown_callback() {
  local exit_code=$?
  exit ${exit_code:-1}
}
# ....Set job name.................................................................................
# TODO: Set DNA_SJOB_NAME
DNA_SJOB_NAME="default"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an DNA_SJOB_NAME.
# ....Python module................................................................................
python_arguments+=("launcher/example.py")
# ====DNA internal=================================================================================
dna_run_slurm_flags+=("--log-name" "$(basename -s .bash $0)")
dna_run_slurm_flags+=("--log-path" "artifact/slurm_jobs_logs")
dna_run_slurm_flags+=("$@")
export DNA_SJOB_NAME
dna::job_setup_callback
trap dna::job_teardown_callback EXIT
EOF

  # Hydra template (v5 state)
  cat > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash" << 'EOF'
#!/bin/bash
#SBATCH --time=0-24:00
#SBATCH --output=out/%x-%j.out

declare -x DNA_SJOB_NAME
# ....Set job name.................................................................................
# TODO: Set DNA_SJOB_NAME
DNA_SJOB_NAME="default"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an DNA_SJOB_NAME.

# ....Hydra app module.............................................................................
hydra_flags+=("launcher/example.py")
# ====DNA internal=================================================================================
dna_run_slurm_flags+=("$@")
export DNA_SJOB_NAME
dna::job_setup_callback
EOF

  # Hydra hparam optim template (v5 state)
  cat > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash" << 'EOF'
#!/bin/bash
#SBATCH --time=0-24:00
#SBATCH --output=out/%x-%j.out

declare -x DNA_SJOB_NAME
# ....Set job name.................................................................................
# TODO: Set DNA_SJOB_NAME
DNA_SJOB_NAME="default"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an DNA_SJOB_NAME.

# ....Hydra app module.............................................................................
hydra_flags+=("launcher/example_app_hparm_optim.py")
# ====DNA internal=================================================================================
dna_run_slurm_flags+=("$@")
export DNA_SJOB_NAME
dna::job_setup_callback
EOF

  # Dryrun template (v5 state)
  mkdir -p "${TEST_TEMP_DIR}/slurm_jobs"
  cat > "${TEST_TEMP_DIR}/slurm_jobs/slurm_job.dryrun.bash" << 'EOF'
#!/bin/bash
#SBATCH --time=0-01:00
#SBATCH --output=out/%x-%j.out

declare -x DNA_SJOB_NAME
# ....Set job name.................................................................................
DNA_SJOB_NAME="dryrun"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an DNA_SJOB_NAME.

# ....Hydra app module.............................................................................
hydra_flags+=("launcher/example_app_hparm_optim.py")
# ====DNA internal=================================================================================
dna_run_slurm_flags+=("$@")
export DNA_SJOB_NAME
dna::job_setup_callback
EOF

  # Valeria apptainer template (v5 state)
  cat > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash" << 'EOF'
#!/bin/bash
#SBATCH --time=0-24:00
#SBATCH --output=out/%x-%j.out

declare -x DNA_SJOB_NAME
# ====Setup========================================================================================
function job_setup_callback() {
  # Add any instruction that should be executed before the apptainer exec command
  module load apptainer

  # Required for wandb.ai
  module load httpproxy

  # Required because configurations under profile.d are not available by default for batch jobs
  # Ref https://doc.s3.valeria.science/fr/calcul/apptainer.html
  source /etc/profile.d/val-utils.sh
}
function job_teardown_callback() {
  local exit_code=$?
  exit "${exit_code:-1}"
}
# ....Set job name.................................................................................
# TODO: Set DNA_SJOB_NAME
DNA_SJOB_NAME="default"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an DNA_SJOB_NAME.
# ....Python module................................................................................
python_arguments+=("launcher/example.py")
# ....HPC server configuration.....................................................................
SUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"
SIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"
PROFILE_ENV_FILE="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria"
# ====DNA internal=================================================================================
export DNA_SJOB_NAME

# Source HPC-specific env (sets DN_PROJECT_PATH, DN_PROJECT_USER, etc.)
# shellcheck source=/dev/null
source "${PROFILE_ENV_FILE}" 2>/dev/null || {
  echo "[warning] Profile env file not found: ${PROFILE_ENV_FILE}" 1>&2
}
# Set Apptainer cache and tmp dirs using Valeria's val-mktemp-dir for best performance
export APPTAINER_CACHEDIR="$( val-mktemp-dir )"
export APPTAINER_TMPDIR="$( val-mktemp-dir )"
EOF

  # Compute Canada apptainer template (v5 state)
  cat > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash" << 'EOF'
#!/bin/bash
#SBATCH --time=0-24:00
#SBATCH --output=out/%x-%j.out

declare -x DNA_SJOB_NAME
# ====Setup========================================================================================
function job_setup_callback() {
  # Add any instruction that should be executed before the apptainer exec command
  :
}
function job_teardown_callback() {
  local exit_code=$?
  exit "${exit_code:-1}"
}
# ....Set job name.................................................................................
# TODO: Set DNA_SJOB_NAME
DNA_SJOB_NAME="default"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an DNA_SJOB_NAME.
# ....Python module................................................................................
python_arguments+=("launcher/example.py")
# ....HPC server configuration.....................................................................
SUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"
SIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"
PROFILE_ENV_FILE="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.compute_canada"
# ====DNA internal=================================================================================
export DNA_SJOB_NAME

# Source HPC-specific env (sets DN_PROJECT_PATH, DN_PROJECT_USER, etc.)
# shellcheck source=/dev/null
source "${PROFILE_ENV_FILE}" 2>/dev/null || {
  echo "[warning] Profile env file not found: ${PROFILE_ENV_FILE}" 1>&2
}
# Set APPTAINER_TMPDIR to SLURM_TMPDIR for best performance on Compute Canada
# (SLURM_TMPDIR is high-speed local storage allocated per job)
APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"
export APPTAINER_TMPDIR
EOF

  # Mock user input to return 'y' (avoids interactive read -r -n 1 prompt in non-interactive bats)
  function dna::patch_prompt_user() {
    REPLY="y"
  }
  export -f dna::patch_prompt_user

  # Mamba apptainer template (v5 state)
  cat > "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash" << 'EOF'
#!/bin/bash
#SBATCH --time=0-24:00
#SBATCH --output=out/%x-%j.out

declare -x DNA_SJOB_NAME
# ====Setup========================================================================================
function job_setup_callback() {
  # Add any instruction that should be executed before the apptainer exec command
  :
}
function job_teardown_callback() {
  local exit_code=$?
  exit "${exit_code:-1}"
}
# ....Set job name.................................................................................
# TODO: Set DNA_SJOB_NAME
DNA_SJOB_NAME="default"
# Note: Recommend opening an issue tracker task (e.g., YouTrack, GitHub issue, Trello)
#  and use its issue ID as an DNA_SJOB_NAME.
# ....Python module................................................................................
python_arguments+=("launcher/example.py")
# ....HPC server configuration.....................................................................
SUPER_PROJECT_ROOT="${SUPER_PROJECT_ROOT:-$(pwd)}"
SIF_PATH="${SIF_PATH:-${SUPER_PROJECT_ROOT}/artifact/apptainer/PLACEHOLDER_DN_PROJECT_IMAGE_NAME-slurm.sif}"
PROFILE_ENV_FILE="${SUPER_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.mamba"
# ====DNA internal=================================================================================
export DNA_SJOB_NAME

# Source HPC-specific env (sets DN_PROJECT_PATH, DN_PROJECT_USER, etc.)
# shellcheck source=/dev/null
source "${PROFILE_ENV_FILE}" 2>/dev/null || {
  echo "[warning] Profile env file not found: ${PROFILE_ENV_FILE}" 1>&2
}
# Set APPTAINER_TMPDIR to SLURM_TMPDIR for best performance on Mamba
APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"
export APPTAINER_TMPDIR
EOF

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

@test "config_scheme_5to6.bash › should update #SBATCH --output path in all slurm templates" {
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
    assert_file_exist "${target_file}"
    run grep "#SBATCH --output=artifact/slurm_jobs_logs/%x-%j.out" "${target_file}"
    assert_success  # New artifact/slurm_jobs_logs path should be present
    run grep "#SBATCH --output=out/%x-%j.out" "${target_file}"
    assert_failure  # Old out/ path should be gone
  done
}

@test "config_scheme_5to6.bash › should skip slurm templates that do not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  rm -rf "${TEST_TEMP_DIR}/slurm_jobs"

  run dna::patch_check_and_run

  assert_success
}

@test "config_scheme_5to6.bash › should remove old DNA_SJOB_NAME=default block from base slurm template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash"
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should add auto-set DNA_SJOB_NAME to DNA internal section in base slurm template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.bash"
  run grep 'basename "${BASH_SOURCE\[0\]}"' "${target_file}"
  assert_success  # auto-set line should be present
  run grep "DNA_SJOB_NAME.*sed.*slurm_job" "${target_file}"
  assert_success  # sed stripping logic should be present
}

@test "config_scheme_5to6.bash › should remove old DNA_SJOB_NAME=default block from hydra template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash"
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should add auto-set DNA_SJOB_NAME to DNA internal section in hydra template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra.bash"
  run grep "hydra\.bash" "${target_file}"
  assert_success  # profile-specific sed stripping .hydra.bash should be present
}

@test "config_scheme_5to6.bash › should remove old DNA_SJOB_NAME=default block from hydra_hparam_optim template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash"
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should add auto-set DNA_SJOB_NAME to DNA internal section in hydra_hparam_optim template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.hydra_hparam_optim.bash"
  run grep "hydra_hparam_optim\.bash" "${target_file}"
  assert_success  # profile-specific sed stripping .hydra_hparam_optim.bash should be present
}

@test "config_scheme_5to6.bash › should remove old DNA_SJOB_NAME=dryrun block from dryrun template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/slurm_job.dryrun.bash"
  run grep 'DNA_SJOB_NAME="dryrun"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should add auto-set DNA_SJOB_NAME to DNA internal section in dryrun template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/slurm_job.dryrun.bash"
  run grep 'basename "${BASH_SOURCE\[0\]}"' "${target_file}"
  assert_success  # auto-set line should be present
}

@test "config_scheme_5to6.bash › should remove old DNA_SJOB_NAME=default block from valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should add auto-set DNA_SJOB_NAME in valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep "apptainer\.valeria\.bash" "${target_file}"
  assert_success  # profile-specific sed stripping .apptainer.valeria.bash should be present
}

@test "config_scheme_5to6.bash › should remove val-mktemp-dir from valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep "val-mktemp-dir" "${target_file}"
  assert_failure  # val-mktemp-dir should have been replaced
}

@test "config_scheme_5to6.bash › should add APPTAINER_CACHEDIR mktemp to valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep 'export APPTAINER_CACHEDIR="\$( mktemp -d' "${target_file}"
  assert_success  # new APPTAINER_CACHEDIR mktemp assignment should be present
}

@test "config_scheme_5to6.bash › should add APPTAINER_TMPDIR mktemp to valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep 'export APPTAINER_TMPDIR="\$( mktemp -d' "${target_file}"
  assert_success  # new APPTAINER_TMPDIR mktemp assignment should be present
}

@test "config_scheme_5to6.bash › should add module spider version-aware load to valeria apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep "module spider apptainer" "${target_file}"
  assert_success  # module spider block should be present
}

@test "config_scheme_5to6.bash › should remove module load apptainer from job_setup_callback in valeria template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  # val-utils.sh should be removed
  run grep "val-utils.sh" "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should keep module load httpproxy in job_setup_callback in valeria template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"
  run grep "module load httpproxy" "${target_file}"
  assert_success  # httpproxy module should still be present
}

@test "config_scheme_5to6.bash › should skip valeria template patches if file does not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  rm -f "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.valeria.bash"

  run dna::patch_check_and_run

  assert_success
}

@test "config_scheme_5to6.bash › should remove old DNA_SJOB_NAME=default block from compute_canada apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should replace APPTAINER_TMPDIR=SLURM_TMPDIR with mktemp in compute_canada template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"

  # Old assignment should be gone
  run grep 'APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"' "${target_file}"
  assert_failure

  # New mktemp-based assignment should be present
  run grep 'export APPTAINER_CACHEDIR="\$( mktemp -d' "${target_file}"
  assert_success
  run grep 'export APPTAINER_TMPDIR="\$( mktemp -d' "${target_file}"
  assert_success
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

@test "config_scheme_5to6.bash › should skip compute_canada template patches if file does not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  rm -f "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.compute_canada.bash"

  run dna::patch_check_and_run

  assert_success
}

@test "config_scheme_5to6.bash › should remove old DNA_SJOB_NAME=default block from mamba apptainer template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"
  run grep 'DNA_SJOB_NAME="default"' "${target_file}"
  assert_failure
}

@test "config_scheme_5to6.bash › should replace APPTAINER_TMPDIR=SLURM_TMPDIR with mktemp in mamba template" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  run dna::patch_check_and_run

  assert_success

  target_file="${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"

  # Old assignment should be gone
  run grep 'APPTAINER_TMPDIR="${SLURM_TMPDIR:-/tmp}"' "${target_file}"
  assert_failure

  # New mktemp-based assignment should be present
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

@test "config_scheme_5to6.bash › should skip mamba template patches if file does not exist" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=5

  rm -f "${TEST_TEMP_DIR}/slurm_jobs/template/slurm_job.DNA_SJOB_NAME.apptainer.mamba.bash"

  run dna::patch_check_and_run

  assert_success
}

@test "config_scheme_5to6.bash › should not apply patch when DNA_CONFIG_SCHEME_VERSION already equals DNA_RELEASE_CONFIG_SCHEME_VERSION" {
  export DNA_RELEASE_CONFIG_SCHEME_VERSION=6
  export DNA_CONFIG_SCHEME_VERSION=6

  run dna::patch_check_and_run

  assert_success
  refute_output --partial "Applying configuration scheme patch: v5 → v6"
}
