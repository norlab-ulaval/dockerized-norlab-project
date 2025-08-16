#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"

unset BUILDX_BUILDER
export DN_ENTRYPOINT_TRACE_EXECUTION=true
export DN_SHOW_DEBUG_INFO=false
_tmp_dir="$(pwd)/$(mktemp)"
echo "_tmp_dir: $_tmp_dir"


function dna::test_teardown_callback() {
  local exit_code=$?
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  unset DN_ENTRYPOINT_TRACE_EXECUTION
  unset DN_SHOW_DEBUG_INFO
#  rm -f "${_tmp_dir}"
  exit ${exit_code:-1}
}
trap dna::test_teardown_callback EXIT

# ====begin========================================================================================
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

_service=deploy
_service_tmp_output="$_tmp_dir/${_service}_output.txt"
_build_all_flag=()
_build_all_flag+=("--service-names" "project-core,project-${_service}")
bash "${DNA_LIB_EXEC_PATH:?err}"/build.all.bash "${_build_all_flag[@]}"

cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
{
  bash "${DNA_LIB_EXEC_PATH:?err}"/run.any.bash --service ${_service} -- bash -c "echo 'Hello world'"
} > "$_service_tmp_output"

test "$(tail -n 1 "$_service_tmp_output")" == "Hello world"
test -n grep -e "project-deploy/dn_entrypoint.init.bash" "$_service_tmp_output"
test -n grep -e "project-deploy/dn_entrypoint.init.callback.bash" "$_service_tmp_output"
