#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"

unset BUILDX_BUILDER

_tmp_output=$(mktemp)
echo "_tmp_output: $_tmp_output"
test -f "$_tmp_output" || exit 1

function dna::test_teardown_callback() {
  local exit_code=$?
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  rm -f "${_tmp_output}"
  exit ${exit_code:-1}
}
trap dna::test_teardown_callback EXIT

# ====begin========================================================================================
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
_build_all_flag=()
_build_all_flag+=("--service-names" "project-core-pre,project-core-user,project-core")
bash "${DNA_LIB_EXEC_PATH:?err}"/build.all.bash "${_build_all_flag[@]}"

_services=( "develop" "deploy" )
for _service in "${_services[@]}" ; do
  _build_all_flag=()
  _build_all_flag+=("--service-names" "project-${_service}")
  bash "${DNA_LIB_EXEC_PATH:?err}"/build.all.bash "${_build_all_flag[@]}"
  test -f "$_tmp_output" || exit 1

  cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
  _run_flag=()
  _run_flag+=("--service" "${_service}")
  _run_flag+=("--" "bash" "-c" "echo 'Hello world'")
  {
    bash "${DNA_LIB_EXEC_PATH:?err}"/run.any.bash "${_run_flag[@]}";
  } | tee "$_tmp_output"

  test -f "$_tmp_output" || { echo "Can't find $_tmp_output!" 1>&2; exit 1; }
  test -n "$(tail "$_tmp_output" | grep "Hello world" )" || { echo "Hello word is missing!" 1>&2; exit 1; }

  echo -e "Completed service $_service ☑️"
done
