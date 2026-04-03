#!/usr/bin/env bats
# =================================================================================================
# Unit tests for apptainer_tools.bash
#
# Test cases:
# - dna::check_apptainer_profile_env_file validation
# - dna::generate_apptainer_build_sif_script generation
# - dna::get_apptainer_slurm_exec_flags output
# - dna::generate_apptainer_run_script generation
# - dna::print_apptainer_exec_command output
#
# Note: These tests run on all platforms including macOS (no apptainer binary required).
#       All functions only GENERATE scripts/commands — they never execute apptainer locally.
#
# =================================================================================================
bats_path=/usr/lib/bats
error_prefix="[\033[1;31mN2ST ERROR\033[0m]"
if [[ -d ${bats_path} ]]; then
  load "${bats_path}/bats-support/load"
  load "${bats_path}/bats-assert/load"
  load "${bats_path}/bats-file/load"
  load "${SRC_CODE_PATH:?err}/${N2ST_BATS_TESTING_TOOLS_RELATIVE_PATH:?err}/bats_helper_functions"
  load "${SRC_CODE_PATH}/tests/tests_bats/bats_testing_tools/bats_helper_functions_local"
else
  echo -e "\n${error_prefix} $0 path to bats-core helper library unreachable at \"${bats_path}\"!"
  echo '(press any key to exit)'
  read -r -n 1
  exit 1
fi
# ====Tests file configuration=====================================================================
TESTED_FILE="apptainer_tools.bash"
TESTED_FILE_PATH="src/lib/core/utils"
# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_DNA_DIR=$(temp_make)

  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils/"

  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" << 'EOF'
#!/bin/bash
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""
export MSG_LINE_CHAR_BUILDER_LVL1="-"
export DNA_ROOT="${MOCK_DNA_DIR}"
export DNA_LIB_PATH="${MOCK_DNA_DIR}/src/lib"
export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
export DN_PROJECT_IMAGE_NAME="test-project"
export PROJECT_TAG="l4t-r36.4.0"
export DN_PROJECT_HUB="norlabulaval"
export DN_PROJECT_GIT_NAME="test-project"

function dna::import_lib_and_dependencies() { return 0; }
function n2st::print_msg() { echo "MSG: $*"; return 0; }
function n2st::print_msg_error() { echo "ERROR: $*" >&2; return 0; }
function n2st::print_msg_done() { echo "DONE: $*"; return 0; }
function n2st::print_msg_warning() { echo "WARNING: $*"; return 0; }
function n2st::draw_horizontal_line_across_the_terminal_window() { echo "---"; return 0; }

for func in $(compgen -A function | grep -e dna:: -e n2st::); do
  export -f "${func}"
done
echo "[dna done] Mock import_dna_lib.bash loaded"
EOF

  export MOCK_PROJECT_ROOT="${MOCK_DNA_DIR}/mock_project"
  mkdir -p "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile"
  echo "DN_PROJECT_PATH=/ros2_ws/src/test-project" \
    > "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria"
}

setup() {
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils"
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" \
    "${MOCK_DNA_DIR}/src/lib/core/utils/"
  source "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1
  export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
  export DN_PROJECT_IMAGE_NAME="test-project"
  export PROJECT_TAG="l4t-r36.4.0"
  cd "${MOCK_DNA_DIR}" || exit 1
}

teardown() {
  bats_print_run_env_variable_on_error
}

teardown_file() {
  temp_del "${MOCK_DNA_DIR}"
}

# ====Tests: dna::check_apptainer_profile_env_file================================================

@test "dna::check_apptainer_profile_env_file with existing profile › expect success" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::check_apptainer_profile_env_file 'valeria'
  "
  assert_success
}

@test "dna::check_apptainer_profile_env_file with missing profile › expect failure" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::check_apptainer_profile_env_file 'nonexistent_profile'
  "
  assert_failure
  assert_output --partial "not found"
}

# ====Tests: dna::generate_apptainer_build_sif_script=============================================

@test "dna::generate_apptainer_build_sif_script › creates build_sif.sh" {
  local output_dir
  output_dir=$(mktemp -d)

  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_build_sif_script \
      'test-project-slurm.l4t-r36.4.0.tar' \
      'test-project-slurm.sif' \
      '${output_dir}'
  "
  assert_success
  assert_file_exists "${output_dir}/build_sif.sh"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › build_sif.sh contains apptainer build command" {
  local output_dir
  output_dir=$(mktemp -d)

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_build_sif_script \
      'test-project-slurm.l4t-r36.4.0.tar' \
      'test-project-slurm.sif' \
      '${output_dir}'
  "

  run grep "apptainer build" "${output_dir}/build_sif.sh"
  assert_success
  assert_output --partial "docker-archive:"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › build_sif.sh is executable" {
  local output_dir
  output_dir=$(mktemp -d)

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_build_sif_script \
      'test-project-slurm.l4t-r36.4.0.tar' \
      'test-project-slurm.sif' \
      '${output_dir}'
  "

  assert_file_executable "${output_dir}/build_sif.sh"

  rm -rf "${output_dir}"
}

# ====Tests: dna::get_apptainer_slurm_exec_flags==================================================

@test "dna::get_apptainer_slurm_exec_flags › output contains --nv flag" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  assert_output --partial "--nv"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --bind flags" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  assert_output --partial "--bind"
  assert_output --partial "artifact/"
  assert_output --partial "data/external_data/"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --env-file with profile" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  assert_output --partial "--env-file"
  assert_output --partial ".env.valeria"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --writable-tmpfs" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  assert_output --partial "--writable-tmpfs"
}

@test "dna::get_apptainer_slurm_exec_flags › output does NOT contain X11 bind mount" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  refute_output --partial ".X11-unix"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --no-eval flag" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  assert_output --partial "--no-eval"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --cleanenv flag" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  assert_output --partial "--cleanenv"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --no-home flag" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  assert_output --partial "--no-home"
}

@test "dna::get_apptainer_slurm_exec_flags with APPTAINER_ENABLE_GPU=true › output contains --nv" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export APPTAINER_ENABLE_GPU=true
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  assert_output --partial "--nv"
}

@test "dna::get_apptainer_slurm_exec_flags with APPTAINER_ENABLE_GPU=false › output does NOT contain --nv" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export APPTAINER_ENABLE_GPU=false
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  refute_output --partial "--nv"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains dynamic SLURM --env vars" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif'
  "
  assert_success
  assert_output --partial "--env CUDA_VISIBLE_DEVICES="
  assert_output --partial "--env SLURM_JOB_ID="
  assert_output --partial "--env SLURM_TMPDIR="
  assert_output --partial "--env SLURM_JOB_NAME="
  assert_output --partial "--env SLURM_NODELIST="
}

# ====Tests: dna::generate_apptainer_run_script===================================================

@test "dna::generate_apptainer_run_script › creates run script file" {
  local output_dir
  output_dir=$(mktemp -d)

  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_IMAGE_NAME='test-project'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_run_script \
      'NMO-001' \
      'valeria' \
      'artifact/apptainer/test-project-slurm.sif' \
      '${output_dir}' \
      'launcher/train.py' '--epochs=10'
  "
  assert_success
  assert_file_exists "${output_dir}/run_apptainer_NMO-001.sh"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_run_script › generated script contains apptainer exec" {
  local output_dir
  output_dir=$(mktemp -d)

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_IMAGE_NAME='test-project'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_run_script \
      'NMO-001' \
      'valeria' \
      'artifact/apptainer/test-project-slurm.sif' \
      '${output_dir}' \
      'launcher/train.py' '--epochs=10'
  "

  run grep "apptainer exec" "${output_dir}/run_apptainer_NMO-001.sh"
  assert_success

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_run_script › generated script contains entrypoint" {
  local output_dir
  output_dir=$(mktemp -d)

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_IMAGE_NAME='test-project'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_run_script \
      'NMO-001' \
      'valeria' \
      'artifact/apptainer/test-project-slurm.sif' \
      '${output_dir}' \
      'launcher/train.py'
  "

  run grep "dn_entrypoint.init.bash" "${output_dir}/run_apptainer_NMO-001.sh"
  assert_success

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_run_script › generated script contains python args" {
  local output_dir
  output_dir=$(mktemp -d)

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_IMAGE_NAME='test-project'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_run_script \
      'NMO-001' \
      'valeria' \
      'artifact/apptainer/test-project-slurm.sif' \
      '${output_dir}' \
      'launcher/train.py' '--epochs=10'
  "

  run grep "launcher/train.py" "${output_dir}/run_apptainer_NMO-001.sh"
  assert_success

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_run_script › generated script is executable" {
  local output_dir
  output_dir=$(mktemp -d)

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_IMAGE_NAME='test-project'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_run_script \
      'NMO-001' \
      'valeria' \
      'artifact/apptainer/test-project-slurm.sif' \
      '${output_dir}' \
      'launcher/train.py'
  "

  assert_file_executable "${output_dir}/run_apptainer_NMO-001.sh"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_run_script › generated script contains Apptainer version warning" {
  local output_dir
  output_dir=$(mktemp -d)

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_IMAGE_NAME='test-project'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_run_script \
      'NMO-001' \
      'valeria' \
      'artifact/apptainer/test-project-slurm.sif' \
      '${output_dir}' \
      'launcher/train.py'
  "

  run grep "Apptainer >= 1.1.0" "${output_dir}/run_apptainer_NMO-001.sh"
  assert_success

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › build_sif.sh contains Apptainer version warning" {
  local output_dir
  output_dir=$(mktemp -d)

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_build_sif_script \
      'test-project-slurm.l4t-r36.4.0.tar' \
      'test-project-slurm.sif' \
      '${output_dir}'
  "

  run grep "Apptainer >= 1.1.0" "${output_dir}/build_sif.sh"
  assert_success

  rm -rf "${output_dir}"
}

# ====Tests: dna::print_apptainer_exec_command====================================================

@test "dna::print_apptainer_exec_command › output starts with apptainer exec" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::print_apptainer_exec_command \
      'valeria' \
      'artifact/apptainer/test-project-slurm.sif' \
      'launcher/train.py'
  "
  assert_success
  assert_output --partial "apptainer exec"
}

@test "dna::print_apptainer_exec_command › output contains entrypoint path" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::print_apptainer_exec_command \
      'valeria' \
      'artifact/apptainer/test-project-slurm.sif' \
      'launcher/train.py'
  "
  assert_success
  assert_output --partial "dn_entrypoint.init.bash"
}

@test "dna::print_apptainer_exec_command with compute_canada profile › contains .env.compute_canada" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::print_apptainer_exec_command \
      'compute_canada' \
      'artifact/apptainer/test-project-slurm.sif' \
      'launcher/train.py'
  "
  assert_success
  assert_output --partial ".env.compute_canada"
}

# ====Tests: dna::load_apptainer_profile_env======================================================

@test "dna::load_apptainer_profile_env with valid DN_PROJECT_USER › exports DN_PROJECT_USER" {
  # Write a profile env file with a valid DN_PROJECT_USER
  echo "DN_PROJECT_USER=hpc_testuser" \
    > "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.testprofile"

  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::load_apptainer_profile_env 'testprofile'
    echo \"DN_PROJECT_USER=\${DN_PROJECT_USER}\"
  "
  assert_success
  assert_output --partial "DN_PROJECT_USER=hpc_testuser"
}

@test "dna::load_apptainer_profile_env with placeholder DN_PROJECT_USER › expect failure" {
  echo "DN_PROJECT_USER=PLACEHOLDER_HPC_USERNAME" \
    > "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.testprofile"

  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::load_apptainer_profile_env 'testprofile'
  "
  assert_failure
  assert_output --partial "DN_PROJECT_USER is not configured"
}

@test "dna::load_apptainer_profile_env with empty DN_PROJECT_USER › expect failure" {
  echo "DN_PROJECT_USER=" \
    > "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.testprofile"

  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::load_apptainer_profile_env 'testprofile'
  "
  assert_failure
  assert_output --partial "DN_PROJECT_USER is not configured"
}

@test "dna::load_apptainer_profile_env with missing profile file › expect failure" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::load_apptainer_profile_env 'nonexistent'
  "
  assert_failure
  assert_output --partial "not found"
}

@test "dna::load_apptainer_profile_env › overrides previously set DN_PROJECT_USER" {
  echo "DN_PROJECT_USER=hpc_jdoe" \
    > "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.testprofile"

  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_USER='local_mac_user'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::load_apptainer_profile_env 'testprofile'
    echo \"DN_PROJECT_USER=\${DN_PROJECT_USER}\"
  "
  assert_success
  assert_output --partial "DN_PROJECT_USER=hpc_jdoe"
}

@test "dna::load_apptainer_profile_env with no DN_PROJECT_USER in profile › expect failure" {
  # Profile that does not set DN_PROJECT_USER at all
  echo "DN_HOST=linux/x86" \
    > "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.testprofile"

  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    unset DN_PROJECT_USER
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::load_apptainer_profile_env 'testprofile'
  "
  assert_failure
  assert_output --partial "DN_PROJECT_USER is not configured"
}
