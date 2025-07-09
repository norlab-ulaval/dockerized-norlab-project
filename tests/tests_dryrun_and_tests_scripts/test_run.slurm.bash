#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_main_dotenv.bash" || exit 1
export PATH="$PATH:${DNA_PATH:?err}"
bash "${DNA_ROOT:?err}/tests/setup_mock.bash"
function dna::test_teardown_callback() {
  exit_code=$?
  if [[ ${exit_code} != 0 ]]; then
    # Make sure there is no slurm container running
    bash "${DNA_LIB_EXEC_PATH:?err}"/down.slurm.bash >/dev/null
  fi
  echo "test_run.slurm.bash DONE"
  cd "${DNA_ROOT:?err}" || exit 1
  bash tests/teardown_mock.bash
  exit ${exit_code:1}
}
trap dna::test_teardown_callback EXIT

cd "${DNA_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================
unset buildx_builder

# Script locals env var
declare -a run_slurm_flags=()
declare -a hydra_flags=()

# Exported env var
declare -x SJOB_ID

SJOB_ID="default"

# ....Debug flags..................................................................................
#run_slurm_flags+=("--hydra-dry-run")
#run_slurm_flags+=(--register-hydra-dry-run-flag "+dev@_global_=math_env_slurm_job_dryrun")
run_slurm_flags+=("--skip-core-force-rebuild")
hydra_flags+=("--version")

bash "${DNA_LIB_EXEC_PATH:?err}"/run.slurm.bash "${SJOB_ID}" "${run_slurm_flags[@]}" "${hydra_flags[@]}"

