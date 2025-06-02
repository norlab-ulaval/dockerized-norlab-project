#!/bin/bash

# ....Setup........................................................................................
source "$(git rev-parse --show-toplevel)/load_repo_dotenv.bash" || exit 1
cd "${DNP_MOCK_SUPER_PROJECT_ROOT:?err}" || exit 1

# ====begin========================================================================================

declare -x SJOB_ID
declare -a FLAGS
declare -a HYDRA_FLAGS

SJOB_ID="default"
# ....Debug flags..................................................................................
#FLAGS+=("--hydra-dry-run")
#FLAGS+=(--register-hydra-dry-run-flag "+dev@_global_=math_env_slurm_job_dryrun")
FLAGS+=("--skip-core-force-rebuild")
HYDRA_FLAGS+=("--version")

bash "${DNP_LIB_EXEC_PATH:?err}"/run.slurm.bash "${SJOB_ID}" "${FLAGS[@]}" "${HYDRA_FLAGS[@]}"
EXIT_CODE=$?

if [[ ${EXIT_CODE} != 0 ]]; then
  # Make sure there is no slurm container running
  bash "${DNP_LIB_EXEC_PATH:?err}"/run_kill.slurm.bash >/dev/null
  exit $EXIT_CODE
fi

echo "test_run.slurm.bash DONE"
