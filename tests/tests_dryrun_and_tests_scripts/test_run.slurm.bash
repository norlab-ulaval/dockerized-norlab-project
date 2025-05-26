#!/bin/bash

# ....path resolution logic........................................................................
SUPER_PROJECT_ROOT=$(git rev-parse --show-toplevel)
cd "${SUPER_PROJECT_ROOT}"/.dockerized_norlab_project/execute

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

bash run.slurm.bash "${SJOB_ID}" "${FLAGS[@]}" "${HYDRA_FLAGS[@]}"
EXIT_CODE=$?

if [[ ${EXIT_CODE} != 0 ]]; then
  # Make sure there is no slurm container running
  bash run_kill.slurm.bash >/dev/null
  exit $EXIT_CODE
fi

echo "test_run.slurm.bash DONE"
