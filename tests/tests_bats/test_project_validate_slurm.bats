#!/usr/bin/env bats
# =================================================================================================
# Unit tests for the helper 'dna::_project_validate_slurm_collect_job_files' used by
# 'dna project validate --slurm'.
#
# The helper filters slurm job script files in a given directory: when 'apptainer' is
# NOT available on the current host, every script matching '*.apptainer.*.bash' is
# routed to the "skipped" array instead of the "included" (to be dry-run) array.
#
# These tests exercise the filter in isolation. They do not invoke docker, compose,
# or the full 'dna::project_validate_slurm' pipeline.
# =================================================================================================

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
  BATS_DOCKER_WORKDIR=$(pwd) && export BATS_DOCKER_WORKDIR
}

setup() {
  export TEST_TEMP_DIR=$(temp_make)
  export TEST_SLURM_DIR="${TEST_TEMP_DIR}/slurm_jobs"
  mkdir -p "${TEST_SLURM_DIR}"

  # The file under test performs a precondition check when sourced (it expects the
  # full DNA + N2ST lib to be loaded). For this unit test we only care about the
  # 'dna::_project_validate_slurm_collect_job_files' helper, so we satisfy the
  # precondition by providing stub functions and the expected SUPER_PROJECT_ROOT.
  function dna::import_lib_and_dependencies() { :; }
  function n2st::print_msg() { :; }
  function dna::build_services() { :; }
  function dna::build_services_multiarch() { :; }
  export -f dna::import_lib_and_dependencies n2st::print_msg \
            dna::build_services dna::build_services_multiarch
  export SUPER_PROJECT_ROOT="${TEST_TEMP_DIR}"

  # Source the file under test.
  # shellcheck disable=SC1091
  source "${BATS_DOCKER_WORKDIR}/src/lib/core/execute/project_validate.slurm.bash"
}

teardown() {
  if [[ -n "${TEST_TEMP_DIR}" && -d "${TEST_TEMP_DIR}" ]]; then
    temp_del "${TEST_TEMP_DIR}"
  fi
  # Clean up any PATH-shadow we may have set up.
  unset _FAKE_APPTAINER_BIN_DIR
}

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

# Stage a fake 'apptainer' executable on PATH so 'command -v apptainer' succeeds.
_stage_fake_apptainer_on_path() {
  export _FAKE_APPTAINER_BIN_DIR="${TEST_TEMP_DIR}/fake_bin"
  mkdir -p "${_FAKE_APPTAINER_BIN_DIR}"
  cat > "${_FAKE_APPTAINER_BIN_DIR}/apptainer" << 'EOF'
#!/bin/bash
echo "fake apptainer $*"
EOF
  chmod +x "${_FAKE_APPTAINER_BIN_DIR}/apptainer"
  export PATH="${_FAKE_APPTAINER_BIN_DIR}:${PATH}"
}

# Force 'command -v apptainer' to fail regardless of the real PATH on this host
# (bats docker images may or may not have apptainer installed).
_hide_apptainer_from_path() {
  # Override 'command' for the current shell so we never resolve apptainer.
  function command() {
    if [[ "$1" == "-v" && "$2" == "apptainer" ]]; then
      return 1
    fi
    builtin command "$@"
  }
  export -f command
}

_restore_command() {
  unset -f command
}

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

@test "dna::_project_validate_slurm_collect_job_files › no slurm job files should return empty arrays" {
  declare -a included=()
  declare -a skipped=()

  _hide_apptainer_from_path
  run dna::_project_validate_slurm_collect_job_files "${TEST_SLURM_DIR}" included skipped
  _restore_command
  assert_success

  # Run again in same shell to actually inspect the arrays.
  _hide_apptainer_from_path
  dna::_project_validate_slurm_collect_job_files "${TEST_SLURM_DIR}" included skipped
  _restore_command

  [[ "${#included[@]}" -eq 0 ]]
  [[ "${#skipped[@]}" -eq 0 ]]
}

@test "dna::_project_validate_slurm_collect_job_files › apptainer not available ⇒ *.apptainer.*.bash are skipped" {
  touch "${TEST_SLURM_DIR}/slurm_job.dryrun.dna.bash"
  touch "${TEST_SLURM_DIR}/slurm_job.train.dna.bash"
  touch "${TEST_SLURM_DIR}/slurm_job.dryrun_pull_and_run.apptainer.valeria.bash"
  touch "${TEST_SLURM_DIR}/slurm_job.other.apptainer.anyhpc.bash"
  # Non-matching file (should be ignored entirely by the glob).
  touch "${TEST_SLURM_DIR}/not_a_slurm_job.bash"

  declare -a included=()
  declare -a skipped=()

  _hide_apptainer_from_path
  dna::_project_validate_slurm_collect_job_files "${TEST_SLURM_DIR}" included skipped
  _restore_command

  # DNA-driven scripts go to 'included'
  [[ " ${included[*]} " == *" slurm_job.dryrun.dna.bash "* ]]
  [[ " ${included[*]} " == *" slurm_job.train.dna.bash "* ]]

  # Apptainer HPC-target scripts go to 'skipped'
  [[ " ${skipped[*]} " == *" slurm_job.dryrun_pull_and_run.apptainer.valeria.bash "* ]]
  [[ " ${skipped[*]} " == *" slurm_job.other.apptainer.anyhpc.bash "* ]]

  # Apptainer scripts must NOT appear in 'included'
  [[ " ${included[*]} " != *".apptainer."* ]]

  # Non-slurm_job file must not appear anywhere.
  [[ " ${included[*]} ${skipped[*]} " != *"not_a_slurm_job.bash"* ]]
}

@test "dna::_project_validate_slurm_collect_job_files › apptainer available ⇒ all slurm_job.*.bash included" {
  touch "${TEST_SLURM_DIR}/slurm_job.dryrun.dna.bash"
  touch "${TEST_SLURM_DIR}/slurm_job.dryrun_pull_and_run.apptainer.valeria.bash"
  touch "${TEST_SLURM_DIR}/slurm_job.other.apptainer.anyhpc.bash"

  declare -a included=()
  declare -a skipped=()

  _stage_fake_apptainer_on_path
  dna::_project_validate_slurm_collect_job_files "${TEST_SLURM_DIR}" included skipped

  # With apptainer on PATH, apptainer-target scripts are NOT filtered out.
  [[ " ${included[*]} " == *" slurm_job.dryrun.dna.bash "* ]]
  [[ " ${included[*]} " == *" slurm_job.dryrun_pull_and_run.apptainer.valeria.bash "* ]]
  [[ " ${included[*]} " == *" slurm_job.other.apptainer.anyhpc.bash "* ]]
  [[ "${#skipped[@]}" -eq 0 ]]
}

@test "dna::_project_validate_slurm_collect_job_files › filter matches '*.apptainer.*.bash' globally (not just valeria)" {
  # Regression guard: the filter must cover ALL HPC profiles, not only valeria.
  touch "${TEST_SLURM_DIR}/slurm_job.foo.apptainer.compute_canada.bash"
  touch "${TEST_SLURM_DIR}/slurm_job.bar.apptainer.narval.bash"
  touch "${TEST_SLURM_DIR}/slurm_job.baz.apptainer.valeria.bash"
  touch "${TEST_SLURM_DIR}/slurm_job.keep.dna.bash"

  declare -a included=()
  declare -a skipped=()

  _hide_apptainer_from_path
  dna::_project_validate_slurm_collect_job_files "${TEST_SLURM_DIR}" included skipped
  _restore_command

  # Only the DNA-driven script is included.
  [[ "${#included[@]}" -eq 1 ]]
  [[ "${included[0]}" == "slurm_job.keep.dna.bash" ]]

  # All three apptainer profile scripts are skipped.
  [[ "${#skipped[@]}" -eq 3 ]]
  [[ " ${skipped[*]} " == *" slurm_job.foo.apptainer.compute_canada.bash "* ]]
  [[ " ${skipped[*]} " == *" slurm_job.bar.apptainer.narval.bash "* ]]
  [[ " ${skipped[*]} " == *" slurm_job.baz.apptainer.valeria.bash "* ]]
}

@test "dna::project_validate_slurm › final summary section reports skipped slurm scripts and reason" {
  # Static regression guard: the final-summary block for skipped jobs must exist
  # in project_validate.slurm.bash so users can tell, at a glance, which slurm
  # scripts were skipped and why.
  local f="${BATS_DOCKER_WORKDIR}/src/lib/core/execute/project_validate.slurm.bash"
  run grep -q 'Skipped slurm job summary' "${f}"
  assert_success
  # The reported reason must mention that 'apptainer' is unavailable on this host.
  run grep -q "apptainer' is not available on this host" "${f}"
  assert_success
  # The summary must iterate the _skipped_apptainer_jobs array.
  run grep -q '_skipped_apptainer_jobs\[idx\]' "${f}"
  assert_success
}
