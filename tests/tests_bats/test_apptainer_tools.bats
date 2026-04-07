#!/usr/bin/env bats
# =================================================================================================
# Unit tests for apptainer_tools.bash
#
# Test cases:
# - dna::check_apptainer_profile_env_file validation
# - dna::generate_apptainer_build_sif_script (dna_tar_to_apptainer_sif_converter.sh) generation
# - dna::get_apptainer_slurm_exec_flags output
# - dna::generate_apptainer_run_script generation
# - dna::print_apptainer_exec_command output
#
# Note: These tests run on all platforms including macOS (no apptainer binary required).
#       All functions only GENERATE scripts/commands — they never execute apptainer locally.
#
# =================================================================================================
bats_require_minimum_version 1.5.0
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

@test "dna::generate_apptainer_build_sif_script › creates dna_tar_to_apptainer_sif_converter.sh" {
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
  assert_file_exists "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh contains apptainer build command" {
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

  run cat "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  assert_success
  assert_output --partial "apptainer build"
  assert_output --partial "docker-archive:"
  assert_output --partial "--mksquashfs-args"
  assert_output --partial "-comp zstd"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh is executable" {
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

  assert_file_executable "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"

  rm -rf "${output_dir}"
}

# ====Tests: dna::get_apptainer_slurm_exec_flags==================================================

@test "dna::get_apptainer_slurm_exec_flags › output contains --nv flag" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
  "
  assert_success
  assert_output --partial "--nv"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --bind flags" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
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
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
  "
  assert_success
  assert_output --partial "--env-file"
  assert_output --partial ".env.valeria"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --writable-tmpfs" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
  "
  assert_success
  assert_output --partial "--writable-tmpfs"
}

@test "dna::get_apptainer_slurm_exec_flags › output does NOT contain X11 bind mount" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
  "
  assert_success
  refute_output --partial ".X11-unix"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --no-eval flag" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
  "
  assert_success
  assert_output --partial "--no-eval"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --cleanenv flag" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
  "
  assert_success
  assert_output --partial "--cleanenv"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains --no-home flag" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
  "
  assert_success
  assert_output --partial "--no-home"
}

@test "dna::get_apptainer_slurm_exec_flags › --nv flag is always unconditionally included" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
  "
  assert_success
  assert_output --partial "--nv"
}

@test "dna::get_apptainer_slurm_exec_flags › APPTAINER_ENABLE_GPU env var is ignored (--nv always included)" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export APPTAINER_ENABLE_GPU=false
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
  "
  assert_success
  # --nv is unconditional; APPTAINER_ENABLE_GPU is no longer used
  assert_output --partial "--nv"
}

@test "dna::get_apptainer_slurm_exec_flags › output contains dynamic SLURM --env vars" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'test-sjob'
  "
  assert_success
  assert_output --partial "--env CUDA_VISIBLE_DEVICES="
  assert_output --partial "--env SLURM_JOB_ID="
  assert_output --partial "--env SLURM_TMPDIR="
  assert_output --partial "--env SLURM_JOB_NAME="
  assert_output --partial "--env SLURM_NODELIST="
}

@test "dna::get_apptainer_slurm_exec_flags › output contains DN_CONTAINER_NAME with sjob name" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::get_apptainer_slurm_exec_flags 'valeria' 'artifact/apptainer/test-project-slurm.sif' 'my-sjob'
  "
  assert_success
  assert_output --partial "--env DN_CONTAINER_NAME="
  assert_output --partial "my-sjob"
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

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh contains Apptainer version warning" {
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

  run grep "Apptainer >= 1.1.0" "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  assert_success

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh deletes tar archive after SIF conversion" {
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

  run grep "rm -f" "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  assert_success
  assert_output --partial "TAR_FILE"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh contains --target-dir argument parsing" {
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

  run grep "\-\-target-dir" "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  assert_success

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh uses TARGET_DIR as SIF output directory when --target-dir is provided" {
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

  # The generated script sets SIF_FILE to <target-dir>/artifact/apptainer/<sif> when --target-dir is used
  run grep "SUPER_PROJECT_ROOT}/artifact/apptainer" "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  assert_success
  assert_output --partial "SIF_FILE"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh fails when --target-dir is provided without argument" {
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

  run bash "${output_dir}/dna_tar_to_apptainer_sif_converter.sh" --target-dir
  assert_failure
  assert_output --partial "--target-dir requires a path argument"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh contains module load apptainer before apptainer build" {
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

  run grep "module load apptainer" "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  assert_success

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh preserves tar archive and exits on apptainer build failure" {
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

  # The generated script must exit on apptainer build failure without deleting the tar
  run grep "Apptainer build failed.*preserved" "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  assert_success
  assert_output --partial "TAR_FILE"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh for valeria profile contains val-mktemp-dir cache config" {
  local output_dir
  output_dir=$(mktemp -d)

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_build_sif_script \
      'test-project-slurm.l4t-r36.4.0.tar' \
      'test-project-slurm.sif' \
      '${output_dir}' \
      'valeria'
  "

  run grep -E "val-mktemp-dir|val-utils|mktemp" "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  assert_success
  assert_output --partial "APPTAINER_CACHEDIR"
  assert_output --partial "APPTAINER_TMPDIR"
  assert_output --partial "source /etc/profile.d/val-utils.sh"
  assert_output --partial "mktemp -d"

  rm -rf "${output_dir}"
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh for non-valeria profile contains mktemp -d cache config" {
  local output_dir
  output_dir=$(mktemp -d)

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_apptainer_build_sif_script \
      'test-project-slurm.l4t-r36.4.0.tar' \
      'test-project-slurm.sif' \
      '${output_dir}' \
      'compute_canada'
  "

  run cat "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  assert_success
  assert_output --partial "APPTAINER_CACHEDIR"
  assert_output --partial "APPTAINER_TMPDIR"
  assert_output --partial "mktemp -d"
  # Non-valeria profile must NOT inject the Valeria-specific val-utils.sh source
  refute_output --partial "source /etc/profile.d/val-utils.sh"

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
      'test-sjob' \
      'launcher/train.py'
  "
  assert_success
  assert_output --partial "apptainer exec"
  assert_output --partial "dn_entrypoint.init.bash"
}

@test "dna::print_apptainer_exec_command › output contains python args" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::print_apptainer_exec_command \
      'valeria' \
      'artifact/apptainer/test-project-slurm.sif' \
      'test-sjob' \
      'launcher/train.py'
  "
  assert_success
  assert_output --partial "launcher/train.py"
}

@test "dna::print_apptainer_exec_command with compute_canada profile › contains .env.compute_canada" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::print_apptainer_exec_command \
      'compute_canada' \
      'artifact/apptainer/test-project-slurm.sif' \
      'test-sjob' \
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

# ====Tests: dna::squash_docker_image=============================================================

# Helper: standard mock docker for the new inspect+export+import+tag squash pipeline
_SQUASH_MOCK_DOCKER='
function docker() {
  case "$1" in
    inspect)
      case "$2" in
        --format={{json\ .Config.Env}})
          echo "[\"ROS_DISTRO=humble\",\"WORKDIR=/app\"]" ;;
        --format={{json\ .Config.Entrypoint}})
          echo "[\"/entrypoints/dn_entrypoint.init.bash\"]" ;;
        --format={{json\ .Config.Cmd}})
          echo "[\"/bin/bash\"]" ;;
        --format={{.Config.WorkingDir}})
          echo "/dockerized-norlab/project/src" ;;
        --format={{.Config.User}})
          echo "ros" ;;
        --format={{json\ .Config.Labels}})
          echo "{\"org.opencontainers.image.authors\":\"test\"}" ;;
        *) echo "mock-inspect" ;;
      esac
      return 0 ;;
    create) echo "mock-container-id-abc123"; return 0 ;;
    export) dd if=/dev/zero bs=1 count=1 2>/dev/null; return 0 ;;
    import) echo "sha256:mockimportedimagesha"; return 0 ;;
    tag)   echo "Mock docker tag: $*"; return 0 ;;
    rm)    echo "Mock docker rm: $*"; return 0 ;;
    rmi)   return 0 ;;
    *)     echo "Mock docker: $*"; return 0 ;;
  esac
}
export -f docker
'

@test "dna::squash_docker_image with valid image › expect success and calls docker inspect/create/export/import/tag" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    ${_SQUASH_MOCK_DOCKER}
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::squash_docker_image 'norlabulaval/test-project-slurm:l4t-r36.4.0'
  "
  assert_success
  assert_output --partial "Squashing Docker image"
  assert_output --partial "squashed successfully"
}

@test "dna::squash_docker_image › metadata preserved in docker import --change flags" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    ${_SQUASH_MOCK_DOCKER}
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    # DNA_DEBUG=true prints the --change flags to stdout so we can assert their contents
    DNA_DEBUG=true dna::squash_docker_image 'norlabulaval/test-project-slurm:l4t-r36.4.0'
  "
  assert_success
  assert_output --partial "ENV ROS_DISTRO="
  assert_output --partial "WORKDIR /dockerized-norlab/project/src"
  assert_output --partial "USER ros"
  assert_output --partial "ENTRYPOINT ["
  assert_output --partial "CMD ["
  assert_output --partial "LABEL org.opencontainers"
}

@test "dna::squash_docker_image › null ENTRYPOINT and CMD are omitted from --change flags" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'

    function docker() {
      case \"\$1\" in
        inspect)
          case \"\$2\" in
            --format={{json\ .Config.Env}})       echo '[]' ;;
            --format={{json\ .Config.Entrypoint}}) echo 'null' ;;
            --format={{json\ .Config.Cmd}})        echo 'null' ;;
            --format={{.Config.WorkingDir}})       echo '' ;;
            --format={{.Config.User}})             echo '' ;;
            --format={{json\ .Config.Labels}})     echo '{}' ;;
            *) echo 'mock-inspect' ;;
          esac
          return 0 ;;
        create) echo 'mock-container-id'; return 0 ;;
        export) dd if=/dev/zero bs=1 count=1 2>/dev/null; return 0 ;;
        import) echo 'sha256:mockimportedimagesha'; return 0 ;;
        tag) return 0 ;;
        rm|rmi) return 0 ;;
        *) return 0 ;;
      esac
    }
    export -f docker

    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    # DNA_DEBUG=true prints the --change flags to stdout so we can assert their contents
    DNA_DEBUG=true dna::squash_docker_image 'norlabulaval/test-project-slurm:l4t-r36.4.0'
  "
  assert_success
  # The --change flags for null/empty metadata should NOT be present
  refute_output --partial "ENTRYPOINT ["
  refute_output --partial "CMD ["
  refute_output --partial "LABEL "
  refute_output --partial "WORKDIR /"
  refute_output --partial "USER "
}

@test "dna::squash_docker_image › docker inspect failure causes function to return error" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'

    function docker() {
      case \"\$1\" in
        inspect)
          echo 'ERROR: No such image' >&2
          return 1 ;;
        *)
          echo \"Mock docker: \$*\"
          return 0 ;;
      esac
    }
    export -f docker

    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::squash_docker_image 'norlabulaval/test-project-slurm:l4t-r36.4.0'
  "
  assert_failure
  assert_output --partial "Failed to inspect image"
}

@test "dna::squash_docker_image › docker create failure causes function to return error" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'

    function docker() {
      case \"\$1\" in
        inspect)
          case \"\$2\" in
            --format={{json\ .Config.Env}}) echo '[]' ;;
            *) echo 'null' ;;
          esac
          return 0 ;;
        create)
          echo 'ERROR: No such image' >&2
          return 1 ;;
        *)
          echo \"Mock docker: \$*\"
          return 0 ;;
      esac
    }
    export -f docker

    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::squash_docker_image 'norlabulaval/test-project-slurm:l4t-r36.4.0'
  "
  assert_failure
  assert_output --partial "Failed to create temporary container"
}

@test "dna::squash_docker_image › docker export failure causes function to return error and cleans up container" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'

    function docker() {
      case \"\$1\" in
        inspect)
          case \"\$2\" in
            --format={{json\ .Config.Env}}) echo '[]' ;;
            *) echo 'null' ;;
          esac
          return 0 ;;
        create)
          echo 'mock-container-id-abc123'
          return 0 ;;
        export)
          echo 'ERROR: export failed' >&2
          return 1 ;;
        rm)
          echo \"Mock docker rm: \$*\"
          return 0 ;;
        *)
          echo \"Mock docker: \$*\"
          return 0 ;;
      esac
    }
    export -f docker

    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::squash_docker_image 'norlabulaval/test-project-slurm:l4t-r36.4.0'
  "
  assert_failure
  assert_output --partial "Failed to export container filesystem"
}

@test "dna::squash_docker_image › docker import failure causes function to return error and cleans up tmp" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'

    function docker() {
      case \"\$1\" in
        inspect)
          case \"\$2\" in
            --format={{json\ .Config.Env}}) echo '[]' ;;
            *) echo 'null' ;;
          esac
          return 0 ;;
        create)
          echo 'mock-container-id-abc123'
          return 0 ;;
        export) dd if=/dev/zero bs=1 count=1 2>/dev/null; return 0 ;;
        import)
          echo 'ERROR: import failed' >&2
          return 1 ;;
        rm|rmi) return 0 ;;
        *)
          echo \"Mock docker: \$*\"
          return 0 ;;
      esac
    }
    export -f docker

    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::squash_docker_image 'norlabulaval/test-project-slurm:l4t-r36.4.0'
  "
  assert_failure
  assert_output --partial "Failed to import squashed image"
}

@test "dna::squash_docker_image › docker tag failure causes function to return error" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    ${_SQUASH_MOCK_DOCKER}

    function docker() {
      case \"\$1\" in
        inspect)
          case \"\$2\" in
            --format={{json\ .Config.Env}}) echo '[]' ;;
            *) echo 'null' ;;
          esac
          return 0 ;;
        create) echo 'mock-container-id-abc123'; return 0 ;;
        export) dd if=/dev/zero bs=1 count=1 2>/dev/null; return 0 ;;
        import) echo 'sha256:mockimportedimagesha'; return 0 ;;
        rm)     echo \"Mock docker rm: \$*\"; return 0 ;;
        tag)
          echo 'ERROR: tag failed' >&2
          return 1 ;;
        rmi)    return 0 ;;
        *)      echo \"Mock docker: \$*\"; return 0 ;;
      esac
    }
    export -f docker

    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::squash_docker_image 'norlabulaval/test-project-slurm:l4t-r36.4.0'
  "
  assert_failure
  assert_output --partial "Failed to re-tag squashed image"
}

@test "dna::squash_docker_image › missing image argument causes error" {
  run -127 bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::squash_docker_image
  "
}

# ====dna::generate_registry_to_apptainer_sif_script tests=========================================

@test "dna::generate_registry_to_apptainer_sif_script › creates dna_registry_to_apptainer_sif_converter.sh" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\"
    test -f \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script is executable" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\"
    test -x \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script contains IMAGE_REF" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\"
    cat \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
  assert_output --partial 'IMAGE_REF="norlabulaval/test-project-slurm:l4t-r36.4.0"'
  assert_output --partial 'SIF_FILENAME="test-project-slurm.sif"'
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script contains apptainer build command" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\"
    cat \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
  assert_output --partial 'apptainer build'
  refute_output --partial 'apptainer build --disable-cache'
  assert_output --partial 'docker://${IMAGE_REF}'
  assert_output --partial '--mksquashfs-args'
  assert_output --partial '-comp zstd'
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script contains --docker-login flag support" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \\
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \\
      'test-project-slurm.sif' \\
      \"\${output_dir}\"
    cat \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
  assert_output --partial '--docker-login'
  assert_output --partial 'USE_DOCKER_LOGIN'
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script contains --target-dir argument parsing" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\"
    cat \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
  assert_output --partial '--target-dir'
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script uses TARGET_DIR as SIF output directory when --target-dir is provided" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\"
    cat \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
  assert_output --partial 'artifact/apptainer'
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script for valeria profile contains val-mktemp-dir cache config" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\" \
      'valeria'
    cat \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
  assert_output --partial 'APPTAINER_CACHEDIR'
  assert_output --partial 'APPTAINER_TMPDIR'
  assert_output --partial 'val-mktemp-dir'
  assert_output --partial 'source /etc/profile.d/val-utils.sh'
  assert_output --partial 'mktemp -d'
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script for non-valeria profile contains mktemp -d cache config" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\" \
      'compute_canada'
    cat \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
  assert_output --partial 'APPTAINER_CACHEDIR'
  assert_output --partial 'APPTAINER_TMPDIR'
  assert_output --partial 'mktemp -d'
  # Non-valeria profile must NOT inject the Valeria-specific val-utils.sh source
  refute_output --partial 'source /etc/profile.d/val-utils.sh'
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script contains module load apptainer" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\"
    cat \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
  assert_output --partial 'module load apptainer'
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script contains error message on build failure" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\"
    cat \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
  assert_output --partial 'Apptainer build failed'
  assert_output --partial '--docker-login'
}

@test "dna::generate_registry_to_apptainer_sif_script › generated script uses APPTAINER_TMPDIR staging before moving SIF to destination" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    output_dir=\$(mktemp -d)
    dna::generate_registry_to_apptainer_sif_script \
      'norlabulaval/test-project-slurm:l4t-r36.4.0' \
      'test-project-slurm.sif' \
      \"\${output_dir}\"
    cat \"\${output_dir}/dna_registry_to_apptainer_sif_converter.sh\"
  "
  assert_success
  assert_output --partial 'SIF_STAGING_DIR'
  assert_output --partial 'SIF_TMP'
  assert_output --partial 'APPTAINER_TMPDIR'
  assert_output --partial 'mv "${SIF_TMP}" "${SIF_FILE}"'
}

@test "dna::generate_apptainer_build_sif_script › dna_tar_to_apptainer_sif_converter.sh uses APPTAINER_TMPDIR staging before moving SIF to destination" {
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

  run cat "${output_dir}/dna_tar_to_apptainer_sif_converter.sh"
  assert_success
  assert_output --partial 'SIF_STAGING_DIR'
  assert_output --partial 'APPTAINER_TMPDIR'
  assert_output --partial 'mv "${SIF_TMP}" "${SIF_FILE}"'

  rm -rf "${output_dir}"
}

@test "dna::generate_registry_to_apptainer_sif_script › missing image_ref argument causes error" {
  run -127 bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    source ${MOCK_DNA_DIR}/src/lib/core/utils/apptainer_tools.bash
    dna::generate_registry_to_apptainer_sif_script
  "
}

