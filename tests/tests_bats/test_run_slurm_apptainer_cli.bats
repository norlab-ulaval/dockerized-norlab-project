#!/usr/bin/env bats
# =================================================================================================
# Unit tests for run.slurm.apptainer.bash CLI parsing
#
# Test cases:
# - dna::run_slurm_apptainer requires sjob-id
# - dna::run_slurm_apptainer requires --apptainer flag
# - dna::run_slurm_apptainer requires python args
# - --print-only flag outputs apptainer exec command to stdout
# - --sif-path flag overrides default SIF path
# - --output-dir flag overrides default output directory
# - generated script is named run_apptainer_<sjob-id>.sh
#
# Note: Tests run on all platforms including macOS (no apptainer binary required).
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
TESTED_FILE="run.slurm.apptainer.bash"
TESTED_FILE_PATH="src/lib/core/execute"
# ....Setup........................................................................................
setup_file() {
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
  export MOCK_DNA_DIR=$(temp_make)

  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/utils/"
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/execute/"

  export MOCK_PROJECT_ROOT="${MOCK_DNA_DIR}/mock_project"
  mkdir -p "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile"
  mkdir -p "${MOCK_PROJECT_ROOT}/artifact/apptainer"
  echo "DN_PROJECT_PATH=/ros2_ws/src/test-project" \
    > "${MOCK_PROJECT_ROOT}/.dockerized_norlab/configuration/hpc_server_profile/.env.valeria"

  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" << 'EOF'
#!/bin/bash
export MSG_DIMMED_FORMAT=""
export MSG_END_FORMAT=""
export MSG_LINE_CHAR_BUILDER_LVL1="-"
export DNA_ROOT="${MOCK_DNA_DIR}"
export DNA_LIB_PATH="${MOCK_DNA_DIR}/src/lib"
export DNA_SPLASH_NAME_FULL="DNA"
export DNA_GIT_REMOTE_URL="https://github.com/test/dna.git"
export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
export DN_PROJECT_IMAGE_NAME="test-project"
export PROJECT_TAG="l4t-r36.4.0"

function dna::import_lib_and_dependencies() { return 0; }
function n2st::print_msg() { echo "MSG: $*"; return 0; }
function n2st::print_msg_error() { echo "ERROR: $*" >&2; return 0; }
function n2st::print_msg_done() { echo "DONE: $*"; return 0; }
function n2st::print_msg_warning() { echo "WARNING: $*"; return 0; }
function n2st::print_formated_script_header() { echo "HEADER: $*"; return 0; }
function n2st::print_formated_script_footer() { echo "FOOTER: $*"; return 0; }
function n2st::norlab_splash() { echo "SPLASH: $*"; return 0; }
function n2st::draw_horizontal_line_across_the_terminal_window() { echo "---"; return 0; }

for func in $(compgen -A function | grep -e dna:: -e n2st::); do
  export -f "${func}"
done
echo "[dna done] Mock import_dna_lib.bash loaded"
EOF

  # Copy real apptainer_tools.bash (we test with real implementation)
  cp "${BATS_DOCKER_WORKDIR}/src/lib/core/utils/apptainer_tools.bash" \
    "${MOCK_DNA_DIR}/src/lib/core/utils/"

  cat > "${MOCK_DNA_DIR}/src/lib/core/utils/load_super_project_config.bash" << EOF
#!/bin/bash
export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
export DN_PROJECT_IMAGE_NAME="test-project"
export PROJECT_TAG="l4t-r36.4.0"
return 0
EOF
}

setup() {
  mkdir -p "${MOCK_DNA_DIR}/src/lib/core/execute"
  cp "${BATS_DOCKER_WORKDIR}/${TESTED_FILE_PATH}/${TESTED_FILE}" \
    "${MOCK_DNA_DIR}/src/lib/core/execute/"
  source "${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash" || exit 1
  export SUPER_PROJECT_ROOT="${MOCK_PROJECT_ROOT}"
  export DN_PROJECT_IMAGE_NAME="test-project"
  cd "${MOCK_DNA_DIR}" || exit 1
}

teardown() {
  bats_print_run_env_variable_on_error
}

teardown_file() {
  temp_del "${MOCK_DNA_DIR}"
}

# ====Tests: CLI argument validation===============================================================

@test "dna::run_slurm_apptainer without sjob-id › expect error" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/execute/run.slurm.apptainer.bash
    dna::run_slurm_apptainer
  "
  assert_failure
}

@test "dna::run_slurm_apptainer without --apptainer flag › expect error" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/execute/run.slurm.apptainer.bash
    dna::run_slurm_apptainer 'NMO-001' 'launcher/train.py'
  "
  assert_failure
  assert_output --partial "--apptainer"
}

@test "dna::run_slurm_apptainer without python args › expect error" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/execute/run.slurm.apptainer.bash
    dna::run_slurm_apptainer 'NMO-001' --apptainer valeria
  "
  assert_failure
  assert_output --partial "python-args"
}

# ====Tests: --print-only mode=====================================================================

@test "dna::run_slurm_apptainer --print-only › outputs apptainer exec to stdout" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_IMAGE_NAME='test-project'
    source ${MOCK_DNA_DIR}/src/lib/core/execute/run.slurm.apptainer.bash
    dna::run_slurm_apptainer 'NMO-001' --apptainer valeria --print-only 'launcher/train.py'
  "
  assert_success
  assert_output --partial "apptainer exec"
}

@test "dna::run_slurm_apptainer --print-only › output contains entrypoint" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_IMAGE_NAME='test-project'
    source ${MOCK_DNA_DIR}/src/lib/core/execute/run.slurm.apptainer.bash
    dna::run_slurm_apptainer 'NMO-001' --apptainer valeria --print-only 'launcher/train.py'
  "
  assert_success
  assert_output --partial "dn_entrypoint.init.bash"
}

# ====Tests: script generation mode================================================================

@test "dna::run_slurm_apptainer › generates run_apptainer_<sjob-id>.sh" {
  local output_dir
  output_dir=$(mktemp -d)

  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_IMAGE_NAME='test-project'
    source ${MOCK_DNA_DIR}/src/lib/core/execute/run.slurm.apptainer.bash
    dna::run_slurm_apptainer 'NMO-001' --apptainer valeria \
      --output-dir '${output_dir}' \
      'launcher/train.py' '--epochs=5'
  "
  assert_success
  assert_file_exists "${output_dir}/run_apptainer_NMO-001.sh"

  rm -rf "${output_dir}"
}

@test "dna::run_slurm_apptainer with --sif-path › script references custom SIF path" {
  local output_dir
  output_dir=$(mktemp -d)
  local custom_sif="/custom/path/myproject.sif"

  bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    export DN_PROJECT_IMAGE_NAME='test-project'
    source ${MOCK_DNA_DIR}/src/lib/core/execute/run.slurm.apptainer.bash
    dna::run_slurm_apptainer 'NMO-001' --apptainer valeria \
      --sif-path '${custom_sif}' \
      --output-dir '${output_dir}' \
      'launcher/train.py'
  "

  run grep "${custom_sif}" "${output_dir}/run_apptainer_NMO-001.sh"
  assert_success

  rm -rf "${output_dir}"
}

@test "dna::run_slurm_apptainer --help › shows help without error" {
  run bash -c "
    source ${MOCK_DNA_DIR}/src/lib/core/utils/import_dna_lib.bash
    export SUPER_PROJECT_ROOT='${MOCK_PROJECT_ROOT}'
    source ${MOCK_DNA_DIR}/src/lib/core/execute/run.slurm.apptainer.bash
    dna::run_slurm_apptainer --help
  "
  assert_success
}
