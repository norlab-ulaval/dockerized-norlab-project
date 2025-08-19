#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"

unset BUILDX_BUILDER

_tmp_output=$(mktemp)
echo "_tmp_output: $_tmp_output"
test -f "$_tmp_output" || exit 1

declare -i test_counter=0
declare -a test_exit_code=()

_services=()
_services+=("develop")
_services+=("deploy")
_services+=("ci-tests")
_services+=("slurm")

function dna::test_teardown_callback() {
  local exit_code=$?
  if [[ ${exit_code} != 0 ]]; then
    # Make sure there is no slurm container running
    bash "${DNA_LIB_EXEC_PATH:?err}"/down.slurm.bash >/dev/null
  fi
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  rm -f "${_tmp_output}"
  echo -e "\nEntrypoints tests results:"
  for service_idx in "${!_services[@]}"; do
    if [[ $service_idx -lt $test_counter ]]; then
      #echo -e "service_idx: $service_idx"
      #echo -e "\${test_exit_code[\$service_idx]}: ${test_exit_code[service_idx]}"
      if [[ -n ${test_exit_code[service_idx]} ]] && [[ ${test_exit_code[service_idx]} -eq 0 ]]; then
        echo -e "  $((service_idx+1))/${#_services[@]} › project-${_services[service_idx]} ✅  "
      else
        echo -e "  $((service_idx+1))/${#_services[@]} › project-${_services[service_idx]} ❌  "
      fi
    else
      echo -e "  $((service_idx+1))/${#_services[@]} › project-${_services[service_idx]} SKIPPED  "
    fi
  done
  echo
  exit ${exit_code:-1}
}
trap dna::test_teardown_callback EXIT

# ====begin========================================================================================
cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
_build_service="$(echo "${_services[@]}" | sed 's/\([^ ]*\)/project-\1/g; s/ /,/g')"
#echo "_build_service: $_build_service"
bash "${DNA_LIB_EXEC_PATH:?err}"/build.all.bash --service-names "${_build_service}"

for _service in "${_services[@]}" ; do
  ((test_counter++))
  echo -e "[test_entrypoints.bash] $test_counter/${#_services[@]} Starting service ${_service}...\n"
  cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1
  _run_flag=()
  _run_flag+=("--env" "DN_ENTRYPOINT_TRACE_EXECUTION=true")
  _run_flag+=("--env" "DN_SHOW_DEBUG_INFO=false")

  if [[ ${_service} =~ ^(develop|deploy)$ ]]; then
    _run_flag+=("--service" "${_service}")
    _run_flag+=("--" "bash" "-c" "echo 'Hello world'")
    {
      bash "${DNA_LIB_EXEC_PATH:?err}"/run.any.bash "${_run_flag[@]}";
    } | tee "$_tmp_output"

    test -n "$(tail "$_tmp_output" | grep "Hello world" )" || { echo "Hello word is missing!" 1>&2; exit 1; }
  elif [[ ${_service} == "ci-tests" ]]; then
    {
      bash "${DNA_LIB_EXEC_PATH:?err}"/run.ci_tests.bash "${_run_flag[@]}";
    } | tee "$_tmp_output"
  elif [[ ${_service} == "slurm" ]]; then
    # Exported env var
    declare -x SJOB_ID
    SJOB_ID="default"

    _run_flag+=("--hydra-dry-run")
    _run_flag+=("--register-hydra-dry-run-flag" "+dev@_global_=math_env_slurm_job_dryrun")
    _run_flag+=("--skip-core-force-rebuild")
    _run_flag+=("--skip-slurm-force-rebuild")
    hydra_flags+=("--version")
    {
      bash "${DNA_LIB_EXEC_PATH:?err}"/run.slurm.bash "${SJOB_ID}" "${_run_flag[@]}" "${hydra_flags[@]}";
    } | tee "$_tmp_output"

  fi

  test -f "$_tmp_output" || { echo "Can't find $_tmp_output!" 1>&2; exit 1; }
  test -n "$(grep -e "\[DN trace\].*Execute" "$_tmp_output")" || { echo "Can't find DN trace retaled promt prefix" 1>&2; exit 1; }
  test -n "$(grep -e "project-${_service}/dn_entrypoint.init.bash" "$_tmp_output")" || { echo "Can't find project-${_service}/dn_entrypoint.init.bash" 1>&2; exit 1; }
  test -n "$(grep -e "project-${_service}/dn_entrypoint.init.callback.bash" "$_tmp_output")" || { echo "Can't find project-${_service}/dn_entrypoint.init.callback.bash" 1>&2; exit 1; }

  test_exit_code+=( 0 )
  echo -e "[test_entrypoints.bash] $test_counter/${#_services[@]} Completed service $_service ☑️\n"
done
