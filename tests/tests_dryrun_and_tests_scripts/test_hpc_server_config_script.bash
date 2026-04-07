#!/bin/bash
# =================================================================================================
# Integration test: dna::generate_hpc_server_config_script — generated script behavior
#
# Calls dna::generate_hpc_server_config_script, then runs the generated dna_hpc_server_config.bash
# against a temporary directory tree to verify:
#   1. All 8 required super-project directories are created.
#   2. The script contains version-aware module load apptainer (module spider + fallback).
#   3. The script contains apptainer registry login --username ... docker://docker.io.
#   4. The --target-dir argument creates the structure under the specified path.
#   5. The script does NOT contain val-utils.sh (generic SLURM_TMPDIR mechanism used instead).
#   6. The script contains --help flag support.
#
# Requires: bash (no Docker, no Apptainer — the script is generated and inspected locally).
# Requires real script generation and execution (fast).
# =================================================================================================
# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"
function dna::test_teardown_callback() {
  local exit_code=$?
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:-1}
}
trap dna::test_teardown_callback EXIT

cd "${N2ST_PATH:?'Variable not set'}" || exit 1
source "import_norlab_shell_script_tools_lib.bash" || exit 1
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
n2st::print_msg "Integration test: dna::generate_hpc_server_config_script"

source "${DNA_ROOT:?err}/src/lib/core/utils/import_dna_lib.bash" || exit 1
source "${DNA_ROOT:?err}/src/lib/core/utils/apptainer_tools.bash" || exit 1

_test_pass=true

_assert_true() {
  local label="$1"
  local result="$2"
  if [[ "${result}" == true ]]; then
    n2st::print_msg_done "  ${label} ✓"
  else
    n2st::print_msg_error "  ${label} FAILED"
    _test_pass=false
  fi
}

# ....Step 1: Generate the script (no profile)..................................................
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Step 1: Generating dna_hpc_server_config.bash (no profile)"

OUTPUT_DIR=$(mktemp -d)
dna::generate_hpc_server_config_script "${OUTPUT_DIR}" || {
  n2st::print_msg_error "dna::generate_hpc_server_config_script failed"
  rm -rf "${OUTPUT_DIR}"
  exit 1
}

SCRIPT="${OUTPUT_DIR}/dna_hpc_server_config.bash"

_assert_true "dna_hpc_server_config.bash exists" "$([[ -f "${SCRIPT}" ]] && echo true || echo false)"
_assert_true "dna_hpc_server_config.bash has bash shebang" "$(head -1 "${SCRIPT}" | grep -q '#!/bin/bash' && echo true || echo false)"

# ....Step 2: Verify script content.............................................................
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Step 2: Verifying generated script content"

for dir in "artifact/apptainer" "artifact/optuna_storage" "artifact/slurm_jobs_logs" \
           "artifact/tensorboard_tmp" "data/external_data" "data/repository_data" \
           "data/shared_data" "slurm_jobs"; do
  _assert_true "contains directory '${dir}'" "$(grep -q "${dir}" "${SCRIPT}" && echo true || echo false)"
done

_assert_true "contains 'module load apptainer'" "$(grep -q 'module load apptainer' "${SCRIPT}" && echo true || echo false)"
_assert_true "contains 'module spider apptainer' (version-aware)" "$(grep -q 'module spider apptainer' "${SCRIPT}" && echo true || echo false)"
_assert_true "contains 'apptainer registry login'" "$(grep -q 'apptainer registry login' "${SCRIPT}" && echo true || echo false)"
_assert_true "contains '--username' in registry login" "$(grep -q '\-\-username' "${SCRIPT}" && echo true || echo false)"
_assert_true "contains 'docker://docker.io'" "$(grep -q 'docker://docker.io' "${SCRIPT}" && echo true || echo false)"
_assert_true "contains '--target-dir'" "$(grep -q '\-\-target-dir' "${SCRIPT}" && echo true || echo false)"
_assert_true "does NOT contain val-utils.sh (generic SLURM_TMPDIR mechanism)" "$(! grep -q 'val-utils.sh' "${SCRIPT}" && echo true || echo false)"
_assert_true "does NOT contain val-mktemp-dir" "$(! grep -q 'val-mktemp-dir' "${SCRIPT}" && echo true || echo false)"
_assert_true "contains '--help' flag support" "$(grep -q '\-\-help' "${SCRIPT}" && echo true || echo false)"

# ....Step 3: Run script with --target-dir and verify directories created.......................
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Step 3: Running dna_hpc_server_config.bash --target-dir and verifying directory creation"

TARGET_DIR=$(mktemp -d)

# Override module and apptainer commands to avoid needing them installed locally
module() { echo "[mock] module $*"; return 0; }
export -f module

apptainer() {
  case "$1" in
    registry) echo "[mock] apptainer registry $*"; return 0 ;;
    *) echo "[mock] apptainer $*"; return 0 ;;
  esac
}
export -f apptainer

echo "testuser" | timeout 10 bash "${SCRIPT}" --target-dir "${TARGET_DIR}" || {
  n2st::print_msg_error "dna_hpc_server_config.bash --target-dir failed"
  rm -rf "${OUTPUT_DIR}" "${TARGET_DIR}"
  exit 1
}

for dir in "artifact/apptainer" "artifact/optuna_storage" "artifact/slurm_jobs_logs" \
           "artifact/tensorboard_tmp" "data/external_data" "data/repository_data" \
           "data/shared_data" "slurm_jobs"; do
  _assert_true "directory created: ${dir}" "$([[ -d "${TARGET_DIR}/${dir}" ]] && echo true || echo false)"
done

rm -rf "${TARGET_DIR}"

# ....Step 4: Verify Valeria profile does NOT inject val-utils.sh (generic mechanism)...........
n2st::draw_horizontal_line_across_the_terminal_window "/"
n2st::print_msg "Step 4: Verifying Valeria profile does NOT inject val-utils.sh (generic SLURM_TMPDIR mechanism)"

OUTPUT_DIR_VALERIA=$(mktemp -d)
dna::generate_hpc_server_config_script "${OUTPUT_DIR_VALERIA}" "valeria" || {
  n2st::print_msg_error "dna::generate_hpc_server_config_script (valeria) failed"
  rm -rf "${OUTPUT_DIR}" "${OUTPUT_DIR_VALERIA}"
  exit 1
}

SCRIPT_VALERIA="${OUTPUT_DIR_VALERIA}/dna_hpc_server_config.bash"
_assert_true "valeria script does NOT contain val-utils.sh (removed in favour of generic SLURM_TMPDIR mechanism)" \
  "$(! grep -q 'val-utils.sh' "${SCRIPT_VALERIA}" && echo true || echo false)"
_assert_true "valeria script does NOT contain val-mktemp-dir" \
  "$(! grep -q 'val-mktemp-dir' "${SCRIPT_VALERIA}" && echo true || echo false)"

rm -rf "${OUTPUT_DIR}" "${OUTPUT_DIR_VALERIA}"

# ....Final result.............................................................................
n2st::draw_horizontal_line_across_the_terminal_window "="
if [[ "${_test_pass}" == true ]]; then
  n2st::print_msg_done "All dna_hpc_server_config.bash assertions passed."
  exit 0
else
  n2st::print_msg_error "One or more dna_hpc_server_config.bash assertions FAILED."
  exit 1
fi
