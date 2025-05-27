#!/bin/bash

# ....Setup........................................................................................
DNP_ROOT=$(git rev-parse --show-toplevel)
DNP_LIB_EXEC_PATH="${DNP_ROOT}/src/lib/core/execute"
SUPER_PROJECT_ROOT="${DNP_ROOT}/mock-user-super-project"
cd "$SUPER_PROJECT_ROOT" || exit 1

# ====begin========================================================================================

declare -x SJOB_ID
declare -a FLAGS
declare -a HYDRA_FLAGS

SJOB_ID="default"
# ....Debug flags..................................................................................
#FLAGS+=("--dry-run")
#FLAGS+=(--register-hydra-dryrun-flag "+dev@_global_=math_env_slurm_job_dryrun")
FLAGS+=("--skip-core-force-rebuild")
HYDRA_FLAGS+=("--version")

bash "${DNP_LIB_EXEC_PATH}"/run.slurm.bash "${SJOB_ID}" "${FLAGS[@]}" "${HYDRA_FLAGS[@]}"
EXIT_CODE=$?

if [[ ${EXIT_CODE} != 0 ]]; then
  # Make sure there is no slurm container running
  bash "${DNP_LIB_EXEC_PATH}"/run_kill.slurm.bash >/dev/null
  exit $EXIT_CODE
fi

echo "test_run.slurm.bash DONE"
